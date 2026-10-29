# SPI and I2S (CH32V203, CH32V303)

This document is for a programmer who needs an SPI bus on a CH32V203 or
a CH32V303: a display, a flash chip or a converter driven from an
active object (a component of a brio program with its own event queue,
served one event at a time by the kernel,
[docs/learn/introduction.md](../learn/introduction.md)), this chip as a
client another host clocks, or, on the CH32V303RC and VC, an I2S audio
link. `Spi<n>` is the resource, a monostate (a class with only static
members, never instantiated: one block, one set of state) with one verb
per register fact. `SpiHost<n, pins, TxEngine, RxEngine,
hold_off_cycles>` is the host engine, the driver layer that moves one
transaction's frames - on the instance's interrupt, on two DMA
channels, or polled - for the bus arbiter of
[util/spi_bus.hpp](../../brio/util/spi_bus.hpp), `SpiBus`: an active
object that owns the bus, runs the requests posted to it one at a time
in the order they arrived, and replies to each with a `SpiDone` event
([design/spi-bus.md](../design/spi-bus.md) is the model). `SpiClient<n,
pins>` is the client, served in polled runs, and `I2s<n>` the same
block's audio face, a resource with nothing above it.

The smallest use, the first block of "How to use it" below - one
device, polled, on SPI2's own column:

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

Line by line: `Bus` names the host engine over SPI2 with every other
template argument at its default - the instance's one column
(`spi_default_pins<2>`: NSS PB12, SCK PB13, MISO PB14, MOSI PB15), no
DMA engines, and the default hold-off (below); SPI2 is a part fact, and
on a CH32V203 below the CH32V203C8, which has SPI1 alone, the line does
not compile. `Bus::init(clock)` takes the clock task's tag object
(`constexpr SysClock clock;`, [clock.md](clock.md)) and no SCK ceiling,
so every rate a request names is legal: it opens the gate, pulses the
block's reset, writes the column, configures a host in mode 0 with
8-bit frames and a software select, hands SCK and MOSI to the
peripheral and takes MISO as an input, raises SPE and enables the
instance's interrupt line at the PFIC, waiting for nothing; it answers
false where a ceiling cannot be made at this clock or the column or the
boot configuration is refused, and the line discards the answer. `cmd`
and `id` are the transaction's buffers, one byte out and three in.
`Bus::Request r{}` is the transaction's descriptor with every field at
its default (no D/C line, no chip-select setup time, an asynchronous
completion) until the lines below set them: `r.cs` is the chip select,
PB12 as a plain GPIO pin (`ref()` makes the pin a value the request
carries), which the engine drives low in `start()` and high again at
the end; `r.cmd` and `r.rx` lend the two buffers
(`brio::lend<brio::Lease::reply>`, a loan that lasts until the reply);
`r.cmd_len = 1` is a command phase of one frame and `r.len = 3` a data
phase of three, which with no out buffer (`r.tx` left null) clocks 0xFF
and keeps what comes back in `id`; `r.mode` and `r.clock` are the device's mode and rate,
the rate a division of the instance's own bus (SPI2 sits on PB1, so
`div16` is 4.5 MHz of SCK under a 144 MHz clock, where PB1
runs at 72 MHz); and `r.polled = true` makes `start()` run the whole
transaction before it returns, spinning with global interrupts enabled
and only this instance's RXNE interrupt silenced, under one spin budget
for the transaction. `start()` then answers true, and `Bus::status()`
says how it ended: `spi_ok`, `spi_stalled` when a flag never came
within the budget (a bus whose clock does not run), or `spi_overrun`
when a frame was lost. These lines run in `main()` after
`SysClock::init()`. A bus that several active objects share goes
through the arbiter instead, which calls `start()` itself - the second
block of "How to use it":

```cpp
using Arb = brio::SpiBus<Bus, Platform, 4>;
BRIO_CH32_VECTOR(spi2_handler) {
    if (Bus::isr()) { brio::post<Arb>(brio::TransferDone{Bus::status()}); }
}
r.polled = false;
r.reply  = brio::reply_to<MyAo, brio::SpiDone>();
brio::post<Arb>(r);
```

`Arb` names the arbiter over `Bus`, an active object listed in the
program's `Tenuto` (`Platform` is the program's platform type,
[platform.md](platform.md)'s `P`), with up to four requests waiting
while one is in flight; a further one is answered `spi_rejected` at
once. `BRIO_CH32_VECTOR(spi2_handler)` binds SPI2's interrupt vector,
under the name the crt (the startup code that runs before `main()`,
`ch32vx03/src/glue/startup_ch32vx03.S`) gives it; its body calls
`Bus::isr()`, the engine's pump, which reads each frame on RXNE and
writes the one after, and which answers true when the transaction has
just completed and the select is high again - the moment the body posts
`TransferDone{Bus::status()}` to the arbiter. `r.polled = false` puts
the transaction on that interrupt, `r.reply` names the active object
(`MyAo`) and the event (`SpiDone`) the answer comes back as, and
`brio::post<Arb>(r)`, from one of `MyAo`'s handlers, copies the request
into the arbiter's queue: the arbiter calls `start()` from its own
dispatch, the frames run on the vector, and `SpiDone`, with the status,
reaches `MyAo` as an event of its own.

What decides correctness even there:

- **The clock task first.** `init()` takes its rates from the clock
  type, not from the hardware: the instance's bus rate
  (`reference_hz()`), against which `sck_hz()` and a ceiling's code are
  worked out, the busy-wait of a request's `cs_setup_us`, and the
  write-ahead codes ("The host engine" below). The rate on the wire is
  the real bus over the request's division, so a host initialized while
  the chip still runs at its 8 MHz HSI reset rate, under a 144 MHz
  type, clocks SPI1's requests eighteen times and SPI2's and
  SPI3's nine times slower than `sck_hz()` reports, and its chip-select
  setup waits eighteen times as long; and on SPI2 and SPI3 the
  write-ahead codes assume a bus at half of HCLK where the reset
  clock's PB1 runs at all of it, so a receive may keep two frames in
  flight at a frame half as long in core cycles as the threshold
  assumed (derived from the arithmetic, not measured). Nothing refuses
  that order: `init()` answers false only for the ceiling, the column
  and the boot configuration.
- **The select pin made an output first.** The engine only stores to
  the request's `cs` (one BSHR or BCR store) and configures nothing, so
  the program drives the pin high and makes it an output
  (`brio::Pin<'B', 12>::output(true)`, [pin.md](pin.md)) before the
  first request - a line neither block writes. A select left as the
  reset leaves it, a floating input, drives nothing.
- **The vector bound by the program**, for a request that is not
  polled: it completes on the instance's interrupt, and a vector left
  unbound runs the crt's weak `default_handler`, a spin loop, at the
  first frame, with no fault and no message. On a host with DMA engines
  a data phase that rides them completes on the receive channel's vector
  instead (`dma_rx_isr()`, the third block of "How to use it"), a polled
  request's included, whose wait in `start()` ends on that vector, while
  the command phase of a request that is not polled still runs on the
  instance's. On the
  CH32V303's images, built with the F extension and the hardware
  prologue on (the core saving the caller-saved integer registers at
  interrupt entry; the project's `CH32VX03_HPE` option, a CMake option
  of the `ch32vx03/` project chosen at configure, on by default),
  `BRIO_CH32_VECTOR` makes a naked trampoline whose body must do no
  floating-point work, which the pump's does not; `brio check ch32vx03`'s
  vector guard fails an image whose trampoline body reaches an FP
  instruction ([platform.md](platform.md)).
- **The buffers lent until the reply.** Through the arbiter, `cmd` and
  `id` stay untouched and in scope from the post to the `SpiDone` event
  `MyAo` receives: a local array of the handler that posts dies with
  the handler, so such buffers are an active object's members or
  static, and nothing checks it. A polled `start()` called directly
  reads and writes them inside the call only.
- **The mode, the rate and the frame size the device wants** travel in
  every request and are checked by nothing but the device; and on a bus
  of loose wires the pads' slew class (`pad_speed()`, the port's fastest
  by default) decides modes 1 and 2 ("A data line's edge is a
  correctness parameter" below).
- **No sleep while an engined block runs.** In a sleep of any depth no
  bus master but the core gets a cycle on this family, so the kernel's
  idle hook returns at once while a DMA channel is enabled (the
  bus-master count, `ch32vx03/bus_activity.hpp`; [sleep.md](sleep.md)),
  and a host with engines holds the program awake while a block is in
  flight.

The rest of this page, in order: the documents of record; what the
silicon does - the instances and their buses, the one buffer each way,
what may be written when, the flags, a data line's edge, the pads, the
DMA slots, the high-speed read and the I2S face; the types and verbs -
the vocabulary, the resource, the host engine and the client; how to use
it, in six blocks; the bench findings - the CH32V203C8T6's, the
CH32V303VCT6's, the pair's, and this part as the desk's instrument
`spi_peer`; and what is not covered.

Up to three synchronous serial ports, both roles, 8- or 16-bit frames,
the four transfer modes, three chip-select arrangements, the simplex and
one-wire line modes, a hardware CRC over the traffic and two DMA
requests each - the STM32F1's SPI under WCH's register names, with one
register of its own (HSCR, a high-speed read mode) - and on the
CH32V303RC and VC the I2S face of the second and third instances.

## Documents of record

The CH32F/V20x_V30x_V31x reference manual V2.3
(20.1 for what the block claims, 20.2 for the SPI face's behaviour,
20.3 for the I2S face's, 20.4 for the registers with tables 20-1 to 20-3
for the three instances' maps and 20.4.10's lot notes for HSCR, 3.3.5.8
and figure 3-3 for the I2S clock, 10.2.11 with tables 10-5, 10-32 and
10-33 for the pads, 11.2.3 with tables 11-2 to 11-5 for the DMA slots,
3.4.7 and 3.4.8 for the gates and 9.5.1 with table 9-2 for the vectors),
the CH32V203 datasheet V2.8 (table 2-1 for how many SPI a part has,
section 3 for which pads it bonds) and the CH32V303 datasheet V3.5
(table 2-1-1 for the SPI and I2S counts, table 3-4 for the audio pads).
Three device classes (WCH's own division of the family, which the
manual keys its chapters and many of its registers by) read the
chapter: CH32V20x_D6 for every CH32V203 up to the CH32V203C8,
CH32V20x_D8 for the CH32V203RB and CH32V30x_D8 for the four CH32V303.
Driver: [brio/ch32vx03/spi.hpp](../../brio/ch32vx03/spi.hpp). Reference
suites: `test_vx03_spi`, whose second set runs against the desk
partner - the CH32V203C8T6's SPI2 wired to the CH32V303VCT6's SPI3 on
its second column, the far board running `spi_peer` - and whose
single-board set uses the CH32V203's SPI1 strap and the CH32V303's
SPI2-SPI3 jumpers, and `test_vx03_i2s` for the audio face. A suite's
letter is one command key of its console and the group of verdicts it
runs: in `test_vx03_spi`, `z` is the key that runs the single-board set
(letters a, b, c, e and f, with h and i on the parts that have SPI3)
and `y` the key that runs the pair's set (d, j, k, l, m, n and o, on
the parts that have the pair's instance), while `x` runs only when
asked for by name; in `test_vx03_i2s`, `z` runs all seven letters.
`bench_vx03`, the stratum's benchmark program (not a test: its letters
print measurements, [design/benchmark.md](../design/benchmark.md)), is
driven by letters the same way.

## What the silicon does

### The instances and their buses - and that is the rate table

SPI1 is the PB2 instance and SPI2 and SPI3 the PB1 ones, which decides
three things at once: the gate bit, the reset line, and - the one that
surprises - the clock BR divides. PB2 runs at HCLK on this family and
PB1 is capped at 72 MHz by this stratum, so at the 144 MHz the bench
runs, `SpiClock::div2` is 72 MHz of SCK on SPI1 and 36 MHz on SPI2 and
SPI3. A program asks the instance for its reference and never the system
clock.

Which instances a part has is its datasheet's table and nothing else,
held as a mask (`device::spi_instances`): every CH32V203 below the
CH32V203C8 carries SPI1 alone, the CH32V203C8 and RB and the 128 KB
CH32V303 carry SPI1 and SPI2, and the CH32V303RC and VC all three. An
instance a part has not got does not compile. SPI3 answers at 0x4000
3C00 with its gate at PB1PCENR's bit 15 and its vector at entry 67 of
the CH32V303's tail.

### No FIFO: one buffer each way

Figure 20-1 is one transmit buffer, one receive buffer and one shift
register. That single fact shapes both roles:

- **TXE means "the buffer moved into the shifter"** - a frame early -
  and **RXNE means "a frame has been shifted both ways"**, which is the
  one moment the bus is provably idle. A host's pump therefore runs on
  RXNE: one write, one interrupt, one read per frame.
- **A client answers one frame ahead.** 20.2.3: the slave starts to
  transmit on the first sampling edge, moving the buffer into the
  shifter and raising TXE. So the first answer must be in the buffer
  before the host's first clock, and every received frame loads the
  next. A client that falls behind does not stall the bus - the host
  reads whatever the shifter held, a wrong answer and never a missing
  one (measured: see the dummy byte below).
- **A client that never drains keeps one frame** and raises the overrun
  for every frame after it (measured).
- **A host may keep two frames in flight, and the receive buffer is the
  price.** 20.2.2: "if the TXE flag bit is set, data needs to be filled
  into the data register to maintain a complete data flow" - one frame in
  the shifter, the next in the transmit buffer, the clock never pausing.
  But the receive buffer is one deep too: frame k stands in it while
  k + 1 shifts, and if k is not read before k + 1 completes, 20.2.7's
  overrun discards one. So a host that reads its answers may run two
  frames ahead only where one frame time exceeds the longest the read can
  be late, and a host that does not read them (a display's pixel path,
  every command phase) may run two ahead at every rate and let the
  receive side overrun - the transmit side goes on regardless (measured:
  the frames sent, judged by the CRC unit, are all there). Measured on
  the CH32V203C8T6 under a live console at 115200 (bench_vx03's letter
  e, the overrun oracle): two frames in flight lose one in every run at
  every frame of 128 core cycles or less, and in none at 256 or more.
- **A DMA-fed transmit with CRCEN set is followed by the CRC frame on the
  wire** (measured, bench_vx03's letter d): the receive block's
  completion lands with BSY still up and RXNE rises once more after it -
  the hardware sends the CRC after the block's last datum with no CRCNEXT
  written, as the F1 lineage's own manual says and 20.2.5 does not. TCRCR
  read while that frame shifts holds a value of the frame in flight;
  20.4.7's "read when BSY is 0" is the rule.

### What may be written, and when

- **DFF and CRCEN "can only be written when SPE is 0"** (20.4.1), which
  is what makes a change of frame size or CRC cost a disable/enable
  pair. BR, LSBFIRST, MSTR, CPOL and CPHA "cannot be modified during
  communication".
- **The disable is a drain**, not a bit: clearing SPE while a frame is
  in flight truncates it on the wire, so the driver waits for TXE and
  for BSY to fall first, each wait bounded.
- **A receive-only host never stops by itself.** With RXONLY set (or
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
| CRCERR | a zero into its bit and ones elsewhere (the one rc_w0 bit) |
| BSY | hardware's, never software's |
| UDR | the I2S face's: a read of STATR (20.3.7.1) |
| CHSIDE | the I2S face's: refreshed by the hardware, never cleared |

**MODF is a demotion**, not a report: the silicon clears SPE and MSTR
with it (measured). That is why this driver's bus engine keeps its chip
select as an ordinary GPIO and never hands the NSS pad to a host. And
20.2.7's recipe for clearing it does not work here - six sequences
measured in this driver and eight more with WCH's own peripheral library
on the same board, its documented one among them, and only the RCC
reset pulse puts the flag down (the findings below list them).

Five interrupt sources over one vector per instance in the SPI face:
TXE and RXNE under their own enables, and CRCERR, OVR and MODF together
under ERRIE; in the I2S face ERRIE gates OVR and UDR.

### A data line's edge is a correctness parameter

On a bus made of jumper wires the fastest pad is the loudest one, and an
edge coupled from a data line into the clock is one sampling edge too
many - which shows in exactly the two modes that sample on the falling
edge while the data lines change on the rising one, modes 1 and 2. So
both tasks (the host engine and the client below, the driver layers with
state of their own over the resource) carry `pad_speed()`, the slew
class of the pads they drive,
and the default is the port's fastest with the measurement in the
findings below.

### The pads are a column

Table 10-32 gives SPI1 two columns - PA4/PA5/PA6/PA7 and
PA15/PB3/PB4/PB5 - selected by one bit of AFIO_PCFR1, and gives SPI2
none: the second instance has no remap field at all and its four pads
(PB12..PB15) come from the datasheet's pin table. Table 10-33 gives SPI3
two behind PCFR1's bit 28: the default PA15/PB3/PB4/PB5 - the very pads
of SPI1's second column - and PA4/PC10/PC11/PC12, and the same bit moves
I2S3's pads with them. Which pads a package brings out is the part's own
table, and three of the CH32V203 parts bond nothing of SPI1's second
column, so naming it there is refused.

### The DMA slots

Tables 11-2 to 11-5, and on this family the channel is the request, on
one of two controllers where the part has two: SPI1 receives on DMA1's
channel 2 and transmits on 3, SPI2 on DMA1's 4 and 5, and SPI3 - and
I2S3, whose requests are the same two - on DMA2's 1 and 2 (a slot is
that pair, a controller and one of its channels). An engine
named on any other slot is refused at compile time, because a wrong one
would move nothing.

One interrupt a transaction: every frame that comes back was clocked out
first, so the receive block's completion proves the transmit block's, and
the host arms its transmit engine for a transfer error alone. The receive
engine arms at VERY_HIGH and the transmit at HIGH
([../design/dma.md](../design/dma.md)): DATAR's receive half holds one
frame, and a receive channel the transmit one outran would find the next
frame on top of its last - OVR, the frame lost (20.2.7) - while a
transmit channel that waits only stops the host's clock between two
frames. The beat - one bus access of a channel - is
the frame: a 16-bit request runs on the engines as half-words - DFF and
the access to DATAR agreeing - when its buffers are half-word aligned
(a misaligned one goes to the pump). And the requests are raised after
both channels are programmed, in one store: a one-frame transaction can
complete before the launch returns, and the old order - each request
raised beside its own channel - raised both again after that completion
and wedged the next transaction ([dma.md](dma.md)).

In sleep the bus matrix serves the core alone on this family
([dma.md](dma.md)), so a transport with engines holds the program awake
while a block is in flight.

### The high-speed read mode: one code means the same on every lot

HSCR.HSRXEN moves a host's sampling of MISO late enough to catch an
answer at the top rate, and 20.4.10 tells its story four ways by class
and by lot. On the CH32V20x_D6 it "is only valid at clock division 2"
and the bit is write-only. On the CH32V20x_D8 - the CH32V203RB - the bit
is write-only and the mode valid at /2 alone on lots whose fifth digit
from the end is below two, anywhere on the others. On the CH32V30x_D8
lots whose penultimate sixth digit is not zero the bit reads back, BR
takes a second ladder under it (20.4.1: FPCLK/2, /3, /4 ... /9 in place
of the powers of two) and a second mode, HSRXEN2, exists - "used only
when HSRXEN is turned on and SPI PCLK is greater than or equal to 120M".
On that class's lots whose sixth digit is zero the bit is write-only
again, and the mode valid at /2 alone where the fifth digit is below
two. BR = 000 is FPCLK/2 in both ladders,
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
them the client's byte one bit late (shifted one place toward the least
significant end with the previous byte's last bit on top: a sample
taken before the answer's edge), and every byte right with it, in both
8- and 16-bit frames; the client received every byte both times. That
die kept no HSRXEN2.

### The I2S face (the CH32V303RC's and VC's)

The register file carries SPIx_I2S_CFGR on every instance and SPIx_I2SPR
on SPI2 and SPI3, and chapter 20 is written for four families at once;
what decides whether a part has the face is its datasheet - table 2-1-1
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
  channel is therefore refused rather than quietly given a 32-bit one.
  A 16-bit datum is one data-register access a channel, a 24- or 32-bit
  one two (the high half first, a 24-bit datum's last byte forced to zero
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
- **TXE reads 1 with I2SE clear**, against 20.3.6.2's "when I2S is
  turned off (I2SE bit is 0), this flag is also 0": on a configured
  transmitting face, before the enable, through it and after the
  disable, TXE read 1 - and a datum written before the enable is the
  first word on the wire once the face is enabled (measured on the
  CH32V303VCT6). So a program may load its first word before it
  enables, and nothing waits for TXE to rise at the enable.
- **The first word is the left channel's.** Under all four standards a
  client receiver's first word was the host's first datum, on the left
  (CHSIDE clear); and a client transmitter fed one datum ahead put that
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
same answer as a compile-time constant, refused rather than reported
where the bus cannot make it. `spi_bus_hz<n>(clock)` is the rate that
instance's BR field divides, out of a static or a dynamic clock tag.

`SpiConfig` collects everything the two control words hold, and
`spi_config_valid` is the chapter's refusals: a field holding a value
its register field does not encode, the CRC asked for outside full
duplex or with a zero polynomial, and a client asking for the NSS output
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
hands back the raised-and-enabled sources and consumes nothing.

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

`SpiHost<n, pins, TxEngine, RxEngine, hold_off_cycles>` is the engine
[util/spi_bus.hpp](../../brio/util/spi_bus.hpp)'s `SpiBus` drives, and
its `Request` carries the fields every family's host shares
([design/spi-bus.md, "Realizations"](../design/spi-bus.md#realizations)),
with the rate and the frame size as this family's `SpiClock` and
`SpiDataSize`: a chip select the engine asserts in `start()`, which the
bus AO calls, a display D/C line, a CS setup time, a command phase and a data
phase with optional out and in buffers, the mode/rate/frame size per
request, and a `polled` flag choosing between the per-frame ISR pump and
a synchronous spin inside `start()`. The Request is laid out with no
padding - the two `PinRef`s, the three buffers and the reply in seven
words, the lengths and the four settings in two (40 bytes where the old
order had 48) - and it is lent for the call: a polled request is read
where it lies and nothing of it is copied; an asynchronous one has what
its tenure (the transaction's time on the bus, from `start()` to its
completion) needs - the pins, the buffers, the lengths, the width -
stored into the engine's own Tenure in word stores, and the reply stays
the arbiter's.

The engine tells two shapes apart by the in buffer. A write (no in
buffer, and every command phase) keeps two frames in flight at every
rate: the polled loop writes on TXE and never reads the answers - the
tail waits TXE and then not BSY, and the DATAR-then-STATR read clears
RXNE and OVR - and the pump reads frame k on RXNE and writes k + 2,
counting an overrun as the frame it is and going on. The pump tests OVR
in a STATR load made after the DATAR one: the frame in flight behind
frame k may complete at any moment before the read, and a status copy
taken before it would miss the OVR that completion raises - frame k + 1
lost uncounted and, on a phase's last pair, no interrupt after it: a
transaction that never completes (measured, below). The load after the
read sees every overrun up to it, and DATAR-then-STATR is 20.2.7's clear,
so the flag goes down in the same handler. A receive (an in
buffer) keeps one frame in flight below the write-ahead threshold and
two above it: the polled loop below it pins the next frame's load inside
the wire time and puts the DATAR read and the next write in adjacent
instructions, the loop above it is 20.2.2's sequence (TXE then the next
frame, RXNE then the previous one) with OVR watched beside RXNE; the
pump below it writes k + 1 on RXNE(k), above it k + 2. An overrun on a
receive ends the transaction with `spi_overrun`, and a polled flag that
never comes within the transaction's one spin budget ends it with
`spi_stalled` - the two codes every host spells from `util/spi_bus.hpp`
beside the engines' own `spi_dma_fault` (design/spi-bus.md).

The threshold is `spi_write_ahead_min_frame_cycles(hold_off)`, the sum
of two inputs: the hold-off - the longest, in core cycles at the clock
the host runs at, that the image keeps the host's vector from running,
which on a platform where no interrupt nests is the longer of the
image's longest handler and its longest masked window - and the pump's
own path from RXNE to the DATAR load (the prologue's 16 and five
instructions, two of them APB loads; the OVR test is a second STATR
load after the DATAR one, below). The hold-off is a fact of the
image and not of the family, so it is the host's template parameter
`hold_off_cycles`, declared by the application or its board file; its
default, 200, is the longest counted in this family's own images
(bench_vx03's console handler with its stamps, 151 instructions behind
the hardware prologue and epilogue, about 200 cycles; the kernel's
`post()` of a Request under the producers' mask, 47 instructions and
the runtime's memcpy, about 130), and a hold-off of zero does not
compile - an image with no handler of its own still has the tick's.
`spi_write_ahead_code<hold_off>(bits, hclk_over_pclk)` turns the sum
into the slowest BR code whose frame is longer, per width and per bus
share, and `rebase()` keeps both codes; `write_ahead_from(bits)`
reports them. Under the default, at any HCLK a frame of 256 cycles is
the first above the sum: SPI1's /32 for 8-bit frames and /16 for
16-bit ones, one code lower on SPI2 and SPI3 above 72 MHz of HCLK; a
declared 300 moves SPI1's to /64 and /32 (both pinned at compile time).
The oracle above agrees with the default. An image measures its own
hold-off with util/bench.hpp's `IsrMeter` in each of its vectors -
`bench_vx03` meters the tick (letter t), the console (p), the DMA's
vectors (d) and the pump (e), `isr` over `irq` a handler's body, the
hardware prologue's entry and exit ([platform.md](platform.md)) on
top - and its longest masked window in its listing; an `spi_overrun` in a
reply is the witness of a figure declared too short.

`apply()` compares one word: the request's mode, rate code and width are
folded into CTLR1's own bits over the applied base (CPHA and CPOL are
`SpiMode`'s two bits in the register's places, BR the code, DFF the
width), compared with the word in force, and written - with the
disable/enable pair 20.4.1 demands - only on a change.

`dma_min_frames` is the data phase at which the engines pay for
themselves: their fixed cost per transaction over the pump's cost per
frame, both from the bench (below); a shorter data phase takes the pump
even with engines bound. Nothing is flushed at `start()`: the chapter
gives no way for a frame to stand in the receive buffer at the start of
a transaction but a tenure that ended without reading what it clocked,
which is a fault path here, and every fault path drains the buffer.

Beside it: `init(clock, max_sck_hz)` with an optional bus-wide SCK
ceiling, `rebase()` for a dynamic clock's fan-out, `clock_for()` /
`sck_hz()` / `reference_hz()`, `prime()` (mode and rate now, moving no
data - a CPOL flip is an edge, so prime before the select falls),
`bit_order()` (a property of the wire and so a bus-level verb),
`pad_speed()` (the slew class of SCK and MOSI, a choice that survives
the next `init()`), `start()`, `isr()`, `dma_rx_isr()` and
`dma_tx_isr()` (the two channels' bodies, each reading its own flags;
`dma_isr()` is both, for a program that binds the two vectors to one
function), `status()`, `busy()`, `recover()`, `release()` and
`claim_nss_pad()`. `release()` closes the gate with SPE cleared and no
reset pulse, unlike the USART's and the I2C host's: on this family a
released USART or I2C holds a DMA request it raised, while SPI1 released
so - an engined write abandoned with TXE standing under TXDMAEN - left
nothing for a timer-paced block on channels 2 and 3 ([dma.md](dma.md),
"A released requester").

### The client

`SpiClient<n, pins>`: `init(clock, Config)`, `enable(first)` (SPE up
with the first answer already in the buffer) and `enable()` (SPE up, the
buffer empty, for a run), `write` / `writable` / `poll`, `selected()` (a
live read of the NSS pad, since the peripheral publishes no select
status), `select()` for a client under software management,
`drive_output()` and `output_driven()` - the dark listener, the answer
line handed to the peripheral only for the window it answers in -
`pad_speed()` for that one pad, the error verbs, `isr()`, `release()`,
and `frames_ahead` = 1, this silicon's answer to the one number a
portable client would need.

The runs are a client's unit, its per-frame verbs the degenerate case:
`answer(out, spins)` (the answers on TXE alone, nothing read),
`exchange(out, in, spins)` (each frame answered one ahead and read on its
RXNE inside the next frame's time) and `receive(in, spins)` (RXNE alone,
nothing written), over `uint8_t` or `uint16_t` frames - anything else is
refused at compile time, DFF having two widths - each returning a `Run`
(the frames moved and a `RunEnd`: complete, closed by the host, or
stalled on the spin budget). They are always inline, so the frame width
and the buffers are constants of the caller's loop. The select pad is the
window: a run waits for it to fall, polls its flag four times a look
and reads the pad once a look, and takes the host's close only from two
looks in a row reading it high (a transient on a board-to-board select
wire is not a close - the findings below). Per frame, the 8-bit exchange
in the release listing is fifteen instructions with four APB accesses
(the two STATR polls, the DATAR store and load) and three taken branches,
on both parts; the 16-bit one the same with half-word memory accesses.

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
BRIO_CH32_VECTOR(spi2_handler) {
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
Fast::init(clock);                         // arms both engines: the gate, DATAR
BRIO_CH32_VECTOR(dma1_channel2_handler) {
    if (Fast::dma_rx_isr()) { /* the transaction completed: Fast::status() */ }
}
BRIO_CH32_VECTOR(dma1_channel3_handler) { (void)Fast::dma_tx_isr(); }

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

The client, answering one frame ahead in a run and dark between windows:

```cpp
using Peer = brio::SpiClient<2>;
Peer::init(clock, {.nss = brio::SpiNss::hardware_input, .drive_output = true});
Peer::pad_speed(brio::PinSpeed::medium); // an edge inside /4's half period
Peer::enable();                          // the buffer empty, the select still high
const auto run = Peer::exchange(std::span<const uint8_t>(answers),
                                std::span<uint8_t>(heard), spins);
if (run.end == Peer::RunEnd::closed) { /* the host closed after run.count frames */ }
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

`test_vx03_spi`'s single-board set on the CH32V203C8T6 at 144 MHz of
HCLK (PB2 = 144 MHz, PB1 = 72 MHz), with a strap from PA7 to PA6 that
makes SPI1's own MOSI its MISO. The CH32V303VCT6's single-board findings
follow in a section of their own, and the pair's - each part against the
other's `spi_peer` - in the one after it.

- **The clock is right on the pad that carries it.** SPI1 on its second
  column puts SCK on PB3, which is TIM2's channel 2 under that timer's
  partial remap, and the pad's own edges reach the capture input while
  the SPI drives it: at every one of the eight BR codes a 64-frame burst
  produced exactly 512 rising edges, the fastest (72 MHz of SCK counted
  by a 144 MHz timer) included.
- **The two instances' rate tables differ**, as their buses do: SPI1's
  ladder runs 72 MHz down to 562.5 kHz, SPI2's 36 MHz down to
  281.25 kHz.
- **THE ENGINES RUN AT THE RATE ASKED** (letter a, no wire, both parts).
  A loop shares one clock at both ends and the edge count above is a
  count, so neither can tell a bus whose rate is not the tree's: a frame
  inside an engined block - the difference of a 2048- and a 256-frame
  block, neither buffer named, the best of three each - is timed on the
  core's counter, which counts HCLK, in thousandths of its SCK periods at
  the instance's own bus over the division. SPI1 read 1000 at /4, /16 and
  /64 and in 16-bit frames at /4, SPI2 1000 at /2 and /16: the clock does
  not pause between frames while the engine refills (20.2.2), and each
  bus divides as the clock tree says. The polled path is never faster
  than its wire at the rate asked - SPI1 on its second column, 64 frames:
  1272 cycles at /2 against the wire's 1024, 2259 at /4 against 2048,
  131283 at /256 against 131072 - nor is letter c's polled ladder on the
  strap, one frame in flight at the top codes (30 cycles a frame at /2
  against 16, 52 at /4 against 32, 2049 at /256 against 2048).
- **THE HOST ABOVE THE WIRE**, `bench_vx03`'s letter e on SPI1 with no
  engines, MISO floating, each line the best of 4 from a fresh tick (the
  wire's cycles a frame: 32 and 128 for 8-bit frames at /4 and /16, 64
  and 256 for 16-bit ones). A POLLED WRITE of 256 frames runs at the
  wire: 32, 128, 64 and 256 cycles a frame (x 1.00 to 1.02), where the
  old loop - one frame in flight, the width reloaded per frame, every
  verb a call - took 101, 197, 137 and 329 (x 1.28 to 3.15); the
  vendor's own loop on the same board (the EVT's 2Lines_FullDuplex
  shape, transcribed into a scratch program on brio's crt and console)
  reads 39, 128, 64 and 256, and at /4 in 8-bit frames LOSES A FRAME IN
  TWO RUNS OF FOUR with nothing live but the tick: a loop that refills on
  TXE whenever it can falls behind a 32-cycle frame (its turn is 39) and
  the receive side overruns, where 20.2.2's sequence - the next write
  after the previous read - cannot fall behind and runs at 34. A POLLED
  RECEIVE, one frame in flight below the threshold, is 51 and 139 cycles
  a frame in 8-bit frames (x 1.62 and 1.09), 79 in 16-bit ones at /4
  (x 1.24), and at /16 in 16-bit frames - two in flight - 256 (x 1.00).
  THE PUMP, one interrupt a frame on a receive: 175 and 242 cycles a
  frame in 8-bit frames (the handler's 103 with its stamps behind the
  frame), 178 at /4 in 16-bit ones, and 257 at /16 (x 1.00, two in
  flight); on a write at /4 one interrupt per TWO frames (the overrun
  counted as the frame it is), 92 cycles a frame against 195 before. A
  POLLED REQUEST OF THREE BYTES - a command and two data, the D/C
  scripted, the select and the D/C on two pads - costs 255 cycles above
  its wire time at /16 (the stopwatch's 28 inside), one byte 197,
  sixteen 249: the price of a DCS command. Of the four pin edges each
  is one BSHR or BCR store inline through `PinRef::set/clear` (pin.hpp
  forces the two verbs inline; as calls they cost the 3-byte request
  25 cycles more, measured). On the CH32V303VCT6 - the V4F, the image
  built under ilp32f, the same letter on the evaluation board's SPI1
  pads with MISO undriven - the shapes land where the CH32V203C8T6's
  do: the polled write of 256 frames at 32, 128, 64 and 256 cycles a
  frame (x 1.00 to 1.02); the polled receive at 43 and 145 cycles a
  frame in 8-bit frames (x 1.36 and 1.13), 79 at /4 in 16-bit ones
  (x 1.24) and 256 at /16 (x 1.00); the pump, its vector the naked
  trampoline of the V4F's images ([platform.md](platform.md)), on a
  receive at 157 and 232 cycles a frame in 8-bit frames, 174 at /4 in
  16-bit ones and 257 at /16 (x 1.00); on a write at /4 one interrupt
  per two frames in 8-bit frames, 83 cycles a frame, and 170 to 171
  interrupts for 256 16-bit frames, 106 cycles a frame - the handler
  entering within a frame time often enough to take one frame at a
  time; the polled request of three bytes 242 cycles above its wire
  time at /16, one byte 196 to 199, sixteen 239 to 246 (three runs);
  every line ended with status 0 and no overrun standing.
- **THE BEAT IS THE FRAME, on the strap** (letter c): sixteen 16-bit
  frames over half-word aligned buffers ride both engines as half-words -
  byte-exact through the loop, ONE channel interrupt for the transaction
  and no SPI one; the same frames over buffers one byte off take the
  pump, sixteen SPI interrupts and no channel one, byte-exact too.
- **A POLLED WAIT THAT RUNS OUT ends `spi_stalled`** (letter f, both
  parts): a polled write of 16 frames started with SPI1's gate closed
  under the engine - every STATR load reading zero, so TXE never comes -
  returned `spi_stalled` after 1596908 to 1596912 cycles on the
  CH32V203C8T6 and 1331207 to 1331239 on the CH32V303VCT6, over the
  budget's 132768 spins: 12.03 and 10.03 cycles a spin of a loop of five
  instructions (the STATR load, the bit test, the budget's decrement and
  test, the branch - poll_write's in the release listing), so the bound
  the suite judges, 5 to 20 cycles a spin, holds on both; the same
  request with the gate open again ran `spi_ok`. Ten runs a part.
- **THE TRANSMIT INTERRUPT ON A HOST** (letter f): SPI1 bare, no pad
  handed over, a vector writing the next frame on each TXE and silencing
  TXEIE after the last - 64 frames at /64 in 65 entries, every frame out
  in order by the CRC unit's own sum (TCRCR 0x1E, the bitwise loop's),
  back to back: 32883 to 32900 cycles on the CH32V203C8T6 and 32891 to
  32908 on the CH32V303VCT6 against the frames' 32768, ten runs a part.
  `txe_interrupt()` and the resource's `isr()` reporting TXE under its
  enable are what it exercises; the host ENGINE still runs on RXNE alone
  (the class comment), this being the shape of a transmit-only stream
  with no reader.
- **THE OVERRUN ORACLE AND THE HOST UNDER IT**: the vendor's
  two-in-flight shape run on the resource at every code and width, 1024
  frames a run, eight runs a point under a console print in flight,
  loses a frame in every run at 32, 64 and 128 cycles a frame and in
  none at 256 and above (five of eight at 16, where a run is too short
  to meet an interrupt); the host under the same load, both shapes, the
  pump and the loop, every code and width, finishes every run with no
  overrun and `spi_ok` - a write tolerating what the oracle counts, a
  receive never put in its way. On the CH32V303VCT6 the same boundary:
  five to eight runs of eight lost a frame at 16 to 128 cycles a frame,
  none at 256 and above, and the host's sixty-four points finished
  every run `spi_ok` with no overrun. ON A PB1 INSTANCE THE BOUNDARY
  MOVES ONE CODE (`spi2.ahead`, `spi2.live`, SPI2 at 72 MHz of bus under
  144 of HCLK, both parts): the oracle lost a frame in five to eight runs
  of eight at 32, 64 and 128 cycles a frame - /2, /4 and /8 in 8-bit
  frames, /2 and /4 in 16-bit ones - and in none at 256 and above, /16 and
  /8; the host, its `write_ahead_from()` reporting /16 and /8, finished its
  sixty-four points `spi_ok` with no overrun - the threshold's arithmetic
  per bus share measured where it differs from SPI1's.
- **A HOLD-OFF DECLARED TOO SHORT LOSES FRAMES, the honest one none**
  (`bench_vx03`'s letter e, `spi.held`, MISO floating, 144 MHz): the
  default host (200 cycles, two frames of a receive in flight from /32)
  and one declaring 100 (from /16), a receive of 1024 8-bit frames at
  /16 - 128 cycles a frame - eight runs each with a console line in
  flight. The POLLED loop, on the CH32V203C8T6 and the CH32V303VCT6
  alike: the honest host `spi_ok` in 8 of 8, the declared-short one
  `spi_overrun` in 6 to 8 runs of 8 on the CH32V303VCT6 over five passes -
  the console's handler, about 200 cycles, outlasting the frame the
  second one in flight left it (5 of 8 on the CH32V203C8T6). The PUMP,
  on both parts: the honest host `spi_ok` in 8 of 8 with no overrun,
  the declared-short one `spi_overrun` in 8 of 8, the witness the class
  comment promises.
  The bench app's selector of the host SPI1's vector serves is
  volatile because gcc, seeing no reader of a plain one between two
  stores of the main path, drops the first as dead: the short host's
  interrupts then go to the plain host, which ends the transaction
  after two entries with `spi_ok` and frames unread (measured: OVR
  standing after 1 to 3 runs of 8, no entry of the short host's branch).
- **THE PUMP READS OVR AFTER THE DATA, and the window that reading it
  before left is closed** (`bench_vx03`'s letter e, `spi.window`, MISO
  floating): a pumped write of 2 and of 3 16-bit frames at /4 and /8,
  two in flight, SPI1's line masked at the PFIC around `start()` and
  unmasked after 0 to 127 empty turns, four sweeps, so that the handler
  for frame 0 enters before frame 1 completes, after it, and on it.
  With OVR tested in a status copy taken BEFORE the DATAR read, a
  completion between the two reads went unseen and the two-frame runs
  that met it never completed - the last frame lost uncounted, no
  interrupt to follow: on the CH32V303VCT6 6 to 9 runs of 512 at /4
  (delays 0 to 3, the earliest the handler can enter) and 4 to 8 at /8
  (delay 28), over four passes; on the CH32V203C8T6 8 at /4 and 4 at
  /8. The three-frame runs all completed, the next frame's RXNE rising
  and its handler finding the OVR a frame late. Under the console's
  interrupts the same window hung `spi.live`'s 16-bit pumped write at /4
  in 1 to 4 runs of 8 on the CH32V303VCT6. With the STATR load after
  the DATAR read: every run of every line completed on both parts, in
  two passes each, no OVR standing after any, and `spi.live` never
  completed 0 at every point; a scratch probe in the handler counted
  the completions that fell between the two reads on the CH32V303VCT6 -
  6 and 7 a line, every one completed. The load costs the handler
  nothing measurable on a receive (89 to 98 cycles a frame with the
  stamps against 92 to 102 on the CH32V303VCT6, the zero-extension
  pair the old copy needed gone with it; 106 to 107 against 106 to 108
  on the CH32V203C8T6) and a few cycles on a write two ahead (106 to
  107 against 103 at /4 in 8-bit frames on the CH32V303VCT6, 119
  against 114 on the CH32V203C8T6).
- **THE TWO-IN-FLIGHT POLLED LOOP WATCHES OVR BESIDE TXE TOO**: in its
  first form it tested OVR on the RXNE poll alone, and `spi.held` found
  the declared-short host's lost frames reported as `spi_stalled` (4 and
  5 runs of 8 in two passes, none as `spi_overrun`), each after the
  transaction's whole spin budget. The loop waits on TXE, so the first
  STATR read after an interrupt that let a frame overrun is the TXE
  poll's - and a STATR read after a DATAR read is 20.2.7's clearing
  sequence, so the flag fell unseen and the loop waited for the frame
  that never came. With OVR tested on every STATR read of the loop: 8
  of 8 `spi_overrun`, at once.
- **THE ENGINED TRANSACTION'S FIXED COST**, `bench_vx03`'s letter d, a
  write on SPI1 with MISO floating at 144 MHz: 530 cycles above the
  frames' own wire time at /4, 523 at /16, ONE interrupt (the receive
  channel's) - the launch 347 of it, the completion's handler 142 with
  its stamps, the instrument 107 - against 571 with the Request copied
  into the host, and 1087 with two interrupts before the engines were
  rewritten ([dma.md](dma.md)). 256 frames at /4 are 8722 cycles against
  8192 of SCK, at /16 33291 against 32768. The engines' own share, 530
  less the instrument, over the pump's 103 cycles a frame is what
  `dma_min_frames` states. On the CH32V303VCT6: 588 to 596 cycles above
  the wire at /4 and 570 to 589 at /16, ONE interrupt - the launch 373
  to 395, the completion's handler 137 to 145 with its stamps - and 256
  frames at /4 in 8785 cycles against 8192, at /16 in 33338 against
  32768 (two runs).
- **16-bit frames through the engines**: a 256-frame write of half-words
  in 16966 cycles at /4 against 16384 of SCK (x 1.03), and 66118 at /16
  against 65536, one interrupt each - the pump took 52545 cycles and 256
  interrupts for the same frames at /4. The CH32V303VCT6: 16972 cycles
  at /4 (x 1.03) and 66125 at /16, one interrupt each.
- **THE TRANSMITTED DATA JUDGED WITH NO WIRE, by the CRC unit**: with
  CRCEN raised under SPE down, TCRCR accumulates every frame the shift
  register sends, so an engined write of 256 frames must leave there what
  a bitwise loop computes over the same buffer - and it did, 0x2E for
  8-bit frames under the polynomial 0x07 and 0xEA0D for 16-bit ones under
  0x1021: the half-word beat, and the order of the two bytes inside a
  frame, are the Request's. The CRC frame follows the block on the wire
  (above), and TCRCR read before it is out holds another number - 0x7D16
  and 0x7E8B in two runs for the 16-bit block, 0xEA0D once BSY is down.
  The CH32V303VCT6's TCRCR held the same 0x2E and 0xEA0D, the CRC frame
  following the block there too.
- **THE STRAP IS CLEAN TO 36 MHz AND BREAKS AT 72.** Every BR code from
  /4 down carried 256 bytes byte-exact; /2 did not. Four kilobytes at
  /4 (36 MHz of SCK), sent as sixteen chunks, came back byte-exact.
- **The pump takes exactly one interrupt per frame** on the strap, 32
  frames byte-exact; the four modes, both widths, both bit orders and the
  CRC are judged by a foreign client (the pair's section).
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
### On the CH32V303VCT6

`test_vx03_spi`'s single-board set and `test_vx03_i2s` (15 verdicts) on
WCH's evaluation board at the same 144 MHz, over four jumper wires
between SPI2's column and SPI3's default one - PB12-PA15, PB13-PB3,
PB14-PB4, PB15-PB5, each looked for before a letter uses it - which carry
I2S2 to I2S3 as well; there is no strap on SPI1's own pads, so letter c
declines by name.

- **The high-speed read** (above): at SPI2's /2, 36 MHz, 254 bytes of
  256 wrong without HSRXEN, 253 of them one bit late, none with it, in
  8282 core cycles for the block against the wire's 8192. SPI1's /2 is
  72 MHz of SCK, reached here through SPI1's second column, whose pads
  are SPI3's default ones and so ride the same wires to SPI2 - and there
  the far end is SPI2 as a CLIENT on a 72 MHz bus, taking a clock at its
  own bus rate: the host read 254 or 255 bytes wrong with the mode and
  without it, and the client itself received 254 to 256 wrong in some
  runs and none in another, so that pairing says nothing about the
  host's read.
  At /8 the same path is exact both ways.
- **The clock counted on PB3 at /2** - 72 MHz of SCK, half the counting
  timer's clock - came to between 233 and 369 of 512 edges in four runs
  on this board, where PB3 carries the jumper to PB13; every slower code
  counted exactly 512.
  Whether the lost edges are the wire's load or the timer's input at its
  own ceiling is not separable here (the CH32V203C8T6, with nothing on
  PB3, counted all 512).
- **Half-words through all four channels** (DMA1's 4 and 5 for SPI2,
  DMA2's 1 and 2 for SPI3, every channel moving 16-bit items into 16-bit
  frames) against the same bytes as 8-bit frames, a 256-byte block each
  way, in core cycles: a 16-bit frame 64.7 at /2 (the wire's 64), 128.7
  at /4, 256.7 at /8 and 512.7 at /16; a byte 32.3, 64.3, 128.3 and 256.3
  cycles in either frame width. The engines add under one per cent to the
  wire at every rate, so a half-word frame halves the number of requests
  and costs nothing on the wire; the block was exact both ways at /4, /8
  and /16 and at /2 with HSRXEN, and one bit late at /2 without it, in
  both widths.
- **THE LINKS RUN AT THE RATE ASKED**: SPI2 and SPI3 both divide PB1, so
  the link between them cannot tell a wrong bus; letters h and i time the
  host's run on the core's counter in thousandths of the wire at PB1 over
  the division: letter h's blocks of 256 frames 1005 at /4 and 1011
  at /2 with HSRXEN; letter i's rows 1011 at /2 in both widths, 1005 at
  /4, 1002 at /8 and 1001 at /16 - the 80 to 90 cycles between the enable
  and the receive channel's completion inside every block, the wire's
  own time at every row.
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

### The pair: the CH32V203C8T6 and the CH32V303VCT6

`test_vx03_spi`'s second set (`y`, letters d, j, k, l, m, n, o) on each board
against the other's `spi_peer`, both at 144 MHz on their crystals: the
CH32V203C8T6's SPI2 (PB12..PB15) wired to the CH32V303VCT6's SPI3 on its
second column (PA4, PC10, PC11, PC12), both selects hardware NSS pads,
over the pair's cable - SCK with 33 Ohm in series at each end and two
grounds of its own, MISO with 56 Ohm at the CH32V203C8T6's PB14, MOSI
with 56 Ohm at the CH32V303VCT6's PC12, the select on its own wire
([../boards/weact-ch32v203c8.md](../boards/weact-ch32v203c8.md),
[../boards/ch32v303-evt.md](../boards/ch32v303-evt.md)) - and every pad
either end drives on it at the MEDIUM slew class. The bounds are
derived in the suite from the two datasheets' SPI tables (the CH32V203's
4-25, the CH32V303's 4-27) and the peer's loops: the far client's hearing
judged from /4 down (a half period holds the 20 ns SCK edge the tables
allow into 30 pF), this end's read-back from /8 down (the client's MISO
valid 25 ns after its edge and the host's 5 ns of setup inside the half
period), the far POLLED run from /16 for 8-bit frames and /8 for 16-bit
ones (two of its turns inside a frame). Ten runs of letters d, j, k, l
and m each way, every verdict green in all twenty; ten runs of n and o
each way, green; `y` whole at 38 verdicts, green each way.

- **THE LADDER, both ways, every host shape**: the polled request, the
  pump and the engines against the far client's DMA serve, and the polled
  request against its polled run, 64 frames a row - exact both ways at
  every code from /256 to /4 = 18 MHz, in both directions, the far run
  included at /4 (above its judged bound). At /2 = 36 MHz THE FAR CLIENT
  HEARS EVERY FRAME, on every shape and in both directions (sixty
  windows of 64 frames each way over the twenty runs, printed - /2 is
  outside the tables' SCK edge), and THIS END READS ITS ANSWERS BACK ONE
  BIT LATE (0xC2 for 0x84 in the first frame: every frame's bits one
  place down, the frame before's last bit on top): the client's
  MISO is valid 25 ns after its edge (tV(SO)) and the host wants 5 ns of
  setup (tSU(MI)), 30 ns against the 13.9 ns of /2's half period - the
  silicon's answer timing, not the wire's. The high-speed read (HSRXEN,
  20.4.10's late sample) brought it back exact on the CH32V303VCT6's host
  in 2 of 10 runs, on the CH32V203C8T6's in none. So the pair's ceiling
  for an exchange is /4, and its write reaches /2 (below). THE
  COST OF EACH HOST SHAPE beside the wire's (cycles a frame, 8-bit, the
  wire 8 x division x 2): the polled request 4099 at /256 (the wire's
  4096), 259 at /16 (256), 153 to 159 at /8 (128, one frame in flight
  below the write-ahead threshold), 87 to 97 at /4 (64); the pump 213 to
  217 at /8 and 150 to 155 at /4; the engines 134 to 135 at /8 and 70 to
  71 at /4 - at the wire from /16 down on every shape.
- **The extended exchange against a foreign unit**: 16-bit frames exact
  both ways on every shape and both far serves (report_extended); the CRC
  at both widths - CRC-8 (0x07) TCRCR 0x62 and RCRCR 0x5B, CRC-16
  (0x8005) 0x5FB6 and 0x2B13, each what a bitwise loop computes over what
  went and what came, the CRC frame read the far unit's own, CRCERR down
  at both ends, and a polynomial mismatch (0x07 here, 0x31 there) raising
  CRCERR at BOTH ends; and this end's WRITE shape - two frames in flight,
  no in buffer - heard exact by a far client that only listens, polled
  and engined, both widths, to /2: the transmit path holds at 36 MHz
  where the answer path does not.
- **THE ROLES INVERT**: each chip's `SpiClient::exchange()` run under the
  far chip's host_burst answers one frame ahead, both ends agreeing frame
  for frame, in 8- and 16-bit frames, and hands its MISO back; each
  part's client in all four modes and both bit orders, answering, and
  in all four receive-only with its MISO dark.
- **THE PADS' SLEW CLASS, THROUGH THE CABLE: EVERY CLASS EXACT.** Each
  host's SCK and MOSI walked at the three classes in all four modes and
  both bit orders (letter d: 8 of 8 at each class in every one of the
  twenty runs), each client's MISO likewise (letter l: 3 of 3 bursts at
  each class, twenty runs), and each part's client ANSWERING in all four
  modes and both orders at the pair's class (letter l, ten runs on each
  part; letter x's sixty cells - 240 exchanges at /256, /64 and /8 on both
  far serves - none bad with the CH32V203C8T6 answering). Over loose wires
  of the same length, with neither the resistors nor the grounds beside
  SCK, a client's own answer edge reached its SCK and select inputs: the
  fast class broke every answer window at 281 kHz, and on the
  CH32V203C8T6's board even the slow class lost frames in modes 1 and 2 -
  the two modes whose answer changes while SCK is high and that sample on
  the falling edge (17 bad cells of letter x's 60) - while a series
  resistor at that board's PB14 alone took it to no bad cell in 180. The pair
  runs every pad at the MEDIUM class: the two datasheets' I/O AC tables
  (the CH32V203's 4-21, the CH32V303's 4-23) give its edge as 25 ns at
  most into 50 pF, inside the 27.8 ns half period of /4, the top judged
  rung, where the slow class's 125 ns fits no rung above /32.
- **THE SELECT WIRE CARRIES THE OTHERS' TRANSIENTS.** Over loose wires,
  on the CH32V203C8T6 as a client, its NSS pad read high for an instant
  inside a window: a
  polled run that took one high read for the host's close ended after 3
  to 17 frames of 32 (and once a DMA serve after 10), while the SPI
  block's own NSS input never saw it (the serves it fed were exact). The
  client's runs therefore close only on two looks in a row reading the
  pad high, which ended it: no early close in the runs since.
- **A LATE FIRST ANSWER GOES INTO THE SHIFTER AT ONCE, AND THE CLIENT
  RESTARTS ITS FRAME THERE** (letter d's dummy, both parts as the far
  client, the same frames from each): an answer written inside the
  host's first frame comes out at the next bit - the first frame reads
  the shifter's leftover bit over the answer's top seven (0x36 for
  0x6D) - and from there the client is ONE BIT behind the host both ways:
  every later answer one bit late (the frame before's last bit on top)
  and the window's last frame never completes (15 heard of 16, each
  wrong). It is not spi_link.hpp's "leftover, then one place late": a
  client too slow for its first answer corrupts the whole window on this
  silicon, which is why the preload (`enable(first)`, or a run started
  with the select high) is the rule.
- **MODF FROM A SECOND CHIP**: a host of the pair with its NSS pad the
  multi-host input, the far chip's ss_pulse pulling the shared select low
  - MODF set within the pulse (CTLR1 0x28 after it: MSTR and SPE gone,
  STATR 0x22), one entry of the instance's vector under ERRIE carrying
  the mode-fault bit, 20.2.7's sequence leaving it standing (the finding
  above, now from a real second host) and the RCC reset pulse clearing
  it.
- **SSOE'S SELECT FALLS WITH THE FIRST FRAME, NOT WITH SPE**: a host with
  SSOE set keeps its NSS pad high under SPE until the first frame is
  written, drives it low from then to the disable, and the far client it
  frames carries sixteen frames exact both ways.
- **`recover()` AFTER A TRANSACTION A SECOND DEVICE KILLED** (letter n):
  a pumped write of 2000 frames at /256 (57 ms) under a TIMED SpiBus (a
  per-bus timeout of 120 ms), this host's NSS pad made the multi-host
  input under the engine, and the far chip's ss_pulse taking the select
  low 30 ms after its ack: MODF 24 to 25 ms into the transaction - SPE
  and MSTR gone, so the pump's RXNE never came and no completion was
  posted - the arbiter's answer `spi_timeout` exactly 120 ms after the
  post, and after `recover()` the block a software-selected host again,
  enabled, MODF down (the reset pulse is the recovery's), the same
  arbiter carrying the next transaction `spi_ok` and the command channel
  answering. Ten runs each way.
- **`rebase()` UNDER A DYNAMICCLOCK** (letter o): the pair's host a user
  of a DynamicClock of 144, 96 and 48 MHz on the HSI's PLL, a ceiling of
  6 MHz declared at init(), each switch's fan-out leaving the engine's
  bus at 72, 48 and 48 MHz, the ceiling at /16, /8 and /8 (4.5, 6 and
  6 MHz), and the write-ahead codes at /16 and /8, /16 and /8, /32 and
  /16 for 8- and 16-bit frames - each what the new bus and its share of
  HCLK make it; and at every rate a request at /2 ran clamped to the
  ceiling, 32 frames exact both ways against the far client in 8404 to
  8431, 5009 to 5190 and 2804 to 2894 cycles (the two parts together)
  against floors of 6144, 4096 and 2048 - no bit shorter than HCLK over
  6 MHz. Ten runs each
  way. The cs_setup timing rebase() also recomputes is not observed by
  the letter.
- **The rest, against a foreign client**: a bit-order mismatch an exact two-way reversal; a client that
  never drains keeps ONE frame and raises OVR; the select a real wire; a
  ten-second stress at 4.5 MHz, polled and engined in turn, 23 exchanges
  of 32 frames exact at both ends.

### This part as the desk's instrument: spi_peer

`ch32vx03/src/apps/spi_peer.cpp`, kind 0x05 of spi_link.hpp, ONE for the
stratum over `PeerLink<instance, column>`: SPI2 where the part has no
SPI3, SPI3 on its second column where it has. Its whole repertoire but
mspi (no USART of these parts reaches the link): the dark listener on
the hardware NSS pad, the exchange on the DMA engines by default and on
the client's polled runs with `spare_polled_pump`, the answer run alone,
THE EXTENDED EXCHANGE whole (16-bit frames, the CRC with CRCNEXT the
store after the last answer's, receive-only with MISO dark), sink_slow,
ss_pulse and host_burst with its own NSS pad a GPIO select, the clock on
the 8 MHz crystal at 144 MHz so that its PB1 instance takes a host's /2.
What the silicon taught it: THE CORE LEAVES THE BUS TO THE ENGINES
inside a DMA serve - a core polling the select pad back to back starved
the receive channel on the CH32V303VCT6 (14 to 31 frames of 64 landed at
18 MHz), polling it once every 256 turns of an SRAM loop kept every
frame ([dma.md](dma.md)); its answer line at the pair's medium slew class; the
close on two reads. The answer path never runs from a vector, so the
CH32V303's float tax does not reach it. Its polled run, in the release
listing, is the client's fifteen instructions a frame; its measured
ceiling behind the polled host is /4 on both parts (the ladder).

## Not covered yet

Driver gaps:

- **The hardware NSS arrangements as the ENGINE's select** (SSOE's
  output and the multi-host input): declined. The bus AO's select is a
  GPIO the Request carries; SSOE's pad falls only with the first frame
  and rises only at the disable, so it frames a whole enable rather than
  a transaction, and the multi-host input takes the host's role away
  (both measured against the far chip, the pair's section).
- **`Spi<2>` as a CLIENT under software management** (SSM with SSI
  driven low by the program instead of by the wire): the peer's select
  is a real wire here, so the software-selected client has no user.
- **A CRC CHECKED over an engined block**: the hardware appends the
  transmit CRC after a DMA-fed block (above), so the frame is there;
  what no verb offers is the receive side's judgement - CRCERR after a
  block whose far end sent its CRC. Born with the first device that
  checks one over a block.
- **The polled loop with two frames in flight under the preemptive
  kernel**: it runs inside the arbiter's dispatch, which every dispatch
  above the arbiter holds off for its whole length, and
  `hold_off_cycles` counts the image's longest handler or masked window,
  not a dispatch. Measured under Rubato on both parts against the other's
  `spi_peer`, with an object preempting every 97 to 389 us
  (`test_vx03_rubato_link`, letter `p`, [platform.md](platform.md)): a
  preemption of 3000 cycles cuts short the rungs whose frame is 2048
  cycles or shorter, one of 300 cycles those of 256 and 512, every time
  with `spi_overrun` and never a run handed back with a hole; the pump
  and the engines are exact at every rung. A hold-off that counts the
  longest dispatch above the arbiter, or a polled path refused under a
  preemptive platform, is the preemptive kernel's open question
  ([../design/kernel.md](../design/kernel.md), "Rubato").
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

- **SPI3's own write-ahead codes**: the same PB1 share as SPI2's, which
  `bench_vx03`'s letter e measures; SPI3's pads on the CH32V303VCT6's
  board are the evaluation board's jumpers and the pair's link, and the
  oracle run on SPI3 would close it by name. The preemptive link suite
  measures the safe side on SPI3 already: two frames in flight from
  256-cycle frames (8-bit at /16, 16-bit at /8) through the pump and the
  polled loop, no overrun in its control passes and the pump exact under
  pressure ([platform.md](platform.md)); the loss on the other side of
  the threshold is measured on SPI1 and SPI2 alone.
- **The high-speed read at 72 MHz of SCK**, SPI1's /2 - the rate the
  mode is for - and **HSRXEN2**: on the CH32V303 board the only far end
  at that rate is SPI2 as a client on a 72 MHz bus, which does not keep
  up, and the die kept no HSRXEN2. A strap from SPI1's MOSI to its own
  MISO, as the CH32V203C8T6's board carries, would measure the first on
  a CH32V303; a CH32V303 of a lot that has the bit, the second.
- **SPI1's clock counted at /2 on a CH32V303**: PB3 carries the jumper
  to PB13 on the evaluation board; the pad free of it would say whether
  the count at half the timer's clock is the silicon's or the wire's.
- **I2S3 on SPI3's second column**: the code is the SPI face's, measured
  as the pair's link; the audio face on it wants a far I2S end, and no
  chip on this desk is one.
- **SPI1's hardware NSS input and output on its own pad.** No letter
  arranges SPI1's select in hardware: on the CH32V203C8T6's board PA4 is
  a free pad, on the CH32V303VCT6's it is the pair's SPI select wire. A
  letter on the CH32V203C8T6 driving SSOE's output on PA4, and a second
  host's select into it, would measure it.
- **`rebase()`'s cs_setup timing**: the delay rate it recomputes is
  written and compiled, and letter o's requests carry no setup time; a
  request with `cs_setup_us` timed from the select's fall to the first
  edge at two rates would measure it.
