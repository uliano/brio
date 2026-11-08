# SPI and I2S (CH32V203, CH32V303)

This document is for a programmer who needs an SPI bus on a CH32V203 or
a CH32V303: a display, a flash chip or a converter driven from an
active object (a component of a brio program with its own event queue,
served one event at a time by the kernel,
[docs/learn/introduction.md](../learn/introduction.md)), this chip as a
client another host clocks, or, on the CH32V303RC and VC, an I2S audio
link. `Spi<n>` is the resource, a monostate (a class with only static
members, never instantiated: one block, one set of state) with one verb
per register fact. `SpiHost<n, pins, TxEngine, RxEngine>` is the host
engine, the driver layer that moves one
transaction's frames - polled, or on two DMA channels - for the bus arbiter of
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
(void)Bus::init(clock);                  // no ceiling, the default column: nothing to refuse

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
r.engine  = brio::SpiEngine::polled;
(void)Bus::start(r);                     // a polled request answers true, done or not: status() decides
if (Bus::status() != brio::spi_ok) { /* spi_stalled: id holds nothing; "What each answer asks of the program" */ }
```

Line by line: `Bus` names the host engine over SPI2 with every other
template argument at its default - the instance's one column
(`spi_default_pins<2>`: NSS PB12, SCK PB13, MISO PB14, MOSI PB15) and no
DMA engines; SPI2 is a part fact, and
on a CH32V203 below the CH32V203C8, which has SPI1 alone, the line does
not compile. `Bus::init(clock)` takes the clock's tag object
(`constexpr SysClock clock;`, [clock.md](clock.md)) and no SCK ceiling,
so every rate a request names is legal: it opens the gate, pulses the
block's reset, writes the column, configures a host in mode 0 with
8-bit frames and a software select, hands SCK and MOSI to the
peripheral and takes MISO as an input and raises SPE, waiting for
nothing and leaving the instance's interrupt line disabled - the host
arms no interrupt of the block; it answers
false where a ceiling cannot be made at this clock or the column or the
boot configuration is refused, and this line, which names no ceiling and
SPI2's own column, discards that answer; a program that names a ceiling
acts on it. `cmd`
and `id` are the transaction's buffers, one byte out and three in.
`Bus::Request r{}` is the transaction's descriptor with every field at
its default (no D/C line, no chip-select setup time, the polled
engine) until the lines below set them: `r.cs` is the chip select,
PB12 as a plain GPIO pin (`ref()` makes the pin a value the request
carries), which the engine drives low in `start()` and high again at
the end; `r.cmd` and `r.rx` lend the two buffers
(`brio::lend<brio::Lease::reply>`, a loan that lasts until the reply);
`r.cmd_len = 1` is a command phase of one frame and `r.len = 3` a data
phase of three, which with no out buffer (`r.tx` left null) clocks 0xFF
and keeps what comes back in `id`; `r.mode` and `r.clock` are the device's mode and rate,
the rate a division of the instance's own bus (SPI2 sits on PB1, so
`div16` is 4.5 MHz of SCK under a 144 MHz clock, where PB1
runs at 72 MHz); and `r.engine = SpiEngine::polled` makes `start()` run the whole
transaction in an explicit loop before it returns, spinning under one
spin budget for the transaction, global interrupts as the caller left
them, depending on no interrupt vector. `start()` then answers true,
and `Bus::status()` says how it ended: `spi_ok`, or `spi_stalled` when a
flag never came within the budget (a bus whose clock does not run), so the
line after it decides whether `id` holds an answer. These lines run in
`main()` after `SysClock::init()`.

A request names the engine that runs it ([design/spi-bus.md, "Two ways
to run a request"](../design/spi-bus.md)), and the host does what it
says at any length: POLLED, as above, or DMA (`r.engine =
SpiEngine::dma`): `start()` launches the instance's two DMA channels and
returns false, and the receive channel's interrupt ends the transaction.
Nothing in the host weighs a request's length; what each engine costs
the core at each rate is measured below ("What each engine costs the
core"), as advice to the program that names one. `Bus` above has no
engines, so it refuses a request naming the engines - `start()` answers
true at once with `spi_rejected`, nothing on the wire - and says so at
compile time (`Bus::has_dma` is false). A bus that several active objects share goes
through the arbiter, which calls `start()` itself, over a host with its
engines - the second block of "How to use it":

```cpp
using Fast = brio::SpiHost<2, brio::spi_default_pins<2>,
                           brio::DmaTxEngine<1, 5>, brio::DmaRxEngine<1, 4>>;
using Arb = brio::SpiBus<Fast, Platform, 4>;
BRIO_CH32_VECTOR(dma1_channel4_handler) {
    if (Fast::dma_rx_isr()) { brio::post<Arb>(brio::TransferDone{Fast::status()}); }
}
BRIO_CH32_VECTOR(dma1_channel5_handler) {
    if (Fast::dma_tx_isr()) { brio::post<Arb>(brio::TransferDone{Fast::status()}); }
}
Fast::Request q{};                       // the fields of r above, then
q.engine = brio::SpiEngine::dma;
q.reply  = brio::reply_to<MyAo, brio::SpiDone>();
brio::post<Arb>(q);
```

`Fast` is SPI2's host with its two DMA engines on the two channels the
reference manual's table 11-5 wires to it (DMA1's channel 5 transmits,
channel 4 receives); `Fast::init(clock)` in `main()` brings it up as
`Bus::init()` did, binds the channels to SPI2's data register and enables their
interrupt lines, so a program calls no DMA set-up of its own.
`Arb` names the arbiter over `Fast`, an active object listed in the
program's `Tenuto` (`Platform` is the program's platform type,
[platform.md](platform.md)'s `P`), with up to four requests waiting
while one is in flight; a further one is answered `spi_rejected` at
once. `BRIO_CH32_VECTOR(dma1_channel4_handler)` binds the receive
channel's interrupt vector, under the name the crt (the startup code
that runs before `main()`, `ch32vx03/src/glue/startup_ch32vx03.S`) gives
it; its body calls `Fast::dma_rx_isr()`, which answers true when the
transaction has just completed and the select is high again - the
moment the body posts `TransferDone{Fast::status()}` to the arbiter. The
transmit channel's vector rings only on a transfer error, and its body
posts the same way. `q.engine = SpiEngine::dma` puts the request on the engines,
`q.reply` names the active object (`MyAo`) and the event (`SpiDone`)
the answer comes back as, and `brio::post<Arb>(q)`, from one of
`MyAo`'s handlers, copies the request into the arbiter's queue: the
arbiter calls `start()` from its own dispatch, the frames run on the
engines, and `SpiDone`, with the status, reaches `MyAo` as an event of
its own, its `status` field the answer ("What each answer asks of the
program" below). Here the one command frame runs in a loop inside
`start()` and the three data frames on the engines: a command phase with
a data phase behind it is the one part of a dma request that runs in the
loop ("The host engine" below). The same four frames at /2, 36 MHz on
SPI2, cost the core fewer cycles polled than on the engines; a program
that would rather spend those cycles than the engines' fixed cost names
`SpiEngine::polled` for them.

What decides correctness even there:

- **The clock first.** `init()` takes its rates from the clock
  type, not from the hardware: the instance's bus rate
  (`reference_hz()`), against which `sck_hz()` and a ceiling's code are
  worked out, and the busy-wait of a request's `cs_setup_us`. The rate on
  the wire is the real bus over the request's division, so a host
  initialized while the chip still runs at its 8 MHz HSI reset rate,
  under a 144 MHz type, clocks SPI1's requests eighteen times and SPI2's
  and SPI3's nine times slower than `sck_hz()` reports, and its
  chip-select setup waits eighteen times as long (derived from the
  arithmetic, not measured). Nothing refuses that order: `init()` answers
  false only for the ceiling, the column and the boot configuration.
- **The select pin made an output first.** The engine only stores to
  the request's `cs` (one BSHR or BCR store) and configures nothing, so
  the program drives the pin high and makes it an output
  (`brio::Pin<'B', 12>::output(true)`, [pin.md](pin.md)) before the
  first request - a line neither block writes. A select left as the
  reset leaves it, a floating input, drives nothing.
- **The two channels' vectors bound by the program**, for a request
  naming the engines: a block completes on the receive
  channel's interrupt, and a vector left unbound runs the crt's weak
  `default_handler`, a spin loop, at the first block's end, with no fault
  and no message. A polled request needs no vector, on any host: it runs
  the loop whatever its length, with interrupts masked as with them
  enabled. The instance's own vector is not the host's: it arms
  no interrupt of the SPI block. On the CH32V303's images, built with the
  F extension and the hardware prologue on (the core saving the
  caller-saved integer registers at interrupt entry; the project's
  `CH32VX03_HPE` option, a CMake option of the `ch32vx03/` project chosen
  at configure, on by default), `BRIO_CH32_VECTOR` makes a naked
  trampoline whose body must do no floating-point work, which the
  channels' bodies do not; `brio check ch32vx03`'s vector guard fails an
  image whose trampoline body reaches an FP instruction
  ([platform.md](platform.md)).
- **A request naming the engines wants engines that can carry it.** On a
  host with none it is refused (`spi_rejected`), so a program that posts
  one asserts `static_assert(Bus::has_dma)`; on a host with them, 16-bit
  frames from an odd address or over engines of `uint8_t` and a run across
  64 KB on the CH32V303's DMA1 are refused the same way, before anything
  moves, and so is any request, either engine's, while a block of the
  host's own is still on them, the one busy check ("The host engine"
  below).
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
(letters a, b, c, e, f, g, s and v, with h and i on the parts that have
SPI3) and `y` the key that runs the pair's set (d, j, k, l, m, n, o, q, r
and t, on the parts that have the pair's instance), while `x` runs only when
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
  one moment the bus is provably idle. A host's receive loop therefore
  reads on RXNE, and the next write follows in the next instruction.
- **A client answers one frame ahead.** 20.2.3: the slave starts to
  transmit on the first sampling edge, moving the buffer into the
  shifter and raising TXE. So the first answer must be in the buffer
  before the host's first clock, and every received frame loads the
  next. A client that falls behind does not stall the bus - the host
  reads whatever the shifter held, a wrong answer and never a missing
  one (measured: see the dummy byte below).
- **A client that never drains keeps one frame** and raises the overrun
  for every frame after it (measured).
- **RXNE rises at the frame's end, in all four modes, never before the
  last SCK edge** (measured: `test_vx03_spi`'s letter s samples STATR and
  the SCK pad together in a masked loop of 31 to 39 cycles a turn, one
  frame at /256 and /64 on every instance; RXNE first read up one to three
  turns after the last sample of SCK away from idle, never on or before it,
  and BSY last read up on that sample or one or two turns after it, on both
  parts; `bench_vx03`'s letter e, `spi.rxne`, on SPI1
  at /256: the write to RXNE 2060 cycles on the CH32V203C8T6 and 2056 to
  2080 on the CH32V303VCT6 against 2048 of wire). 20.2.2 says RXNE is set
  when the last sampling edge arrives, which in modes 0 and 2 is half a
  bit, 128 cycles on SPI1 at /256, before the frame's end; on this silicon
  nothing shows that half bit. So WHEN A REQUEST ENDS THE WIRE HAS ENDED
  without a wait of the host's own: the polled write's tail waits BSY's
  fall, the polled read's last frame and the receive channel's completion
  rest on RXNE, and the select rises after the last edge on both engines
  ("Bench findings", letter s).
- **The transmit buffer's write-ahead: used by the transmit-only loop,
  declined by the receive loop.** 20.2.2: "if the TXE flag bit is set, data
  needs to be filled into the data register to maintain a complete data
  flow" - one frame in the shifter, the next in the transmit buffer, the
  clock never pausing. A host that does not read its answers (a display's
  pixel path, every command phase) writes ahead at every rate and lets the
  receive side overrun; the transmit side goes on regardless (measured:
  the frames sent, judged by the CRC unit, are all there). But the receive
  buffer is one deep too: with a second frame queued behind the one
  shifting, frame k stands in it while k + 1 shifts, and if k is not read
  before k + 1 completes, 20.2.7's overrun discards one. Measured on the
  resource under a live console at 115200 (`bench_vx03`'s letter e,
  `spi.ahead`, the vendor's loop): a frame lost at every frame
  of 32, 64 or 128 core cycles in seven or eight runs of eight, in none at
  256 or more - and no bound on an
  image's handlers, or on the dispatches above a preempted one, can be
  enforced here. So the host's receive loop keeps one frame in flight at
  every rate: the clock waits for the read, and a handler or a preemption
  is a gap on the bus. The price is the turn between two frames, at most
  22 cycles a frame on SPI1 and 32 on SPI2 at 144 MHz
  ("Bench findings"); a read that wants the wire's own rate names the
  engines, and they keep it.
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
| OVR | a DATAR read followed by a STATR read, which still reads it up; the load after it reads it down (letter v) |
| MODF | a STATR read followed by a CTLR1 write with nothing between them (measured below); the block's RCC reset pulse |
| CRCERR | a zero into its bit and ones elsewhere (the one rc_w0 bit), measured on a flag the hardware raised (letter e, below) |
| BSY | hardware's, never software's |
| UDR | the I2S face's: a read of STATR (20.3.7.1) |
| CHSIDE | the I2S face's: refreshed by the hardware, never cleared |

**MODF is a demotion**, not a report: the silicon clears SPE and MSTR
with it (measured). That is why this driver's bus engine keeps its chip
select as an ordinary GPIO and never hands the NSS pad to a host. And
20.2.7's recipe for clearing it is stricter on the CH32V203C8T6 than its
words: with nothing between the STATR read and the CTLR1 write it
clears the flag, and a read of CTLR1 or CTLR2 between the two steps
leaves the flag standing there, where the CH32V303CBT6 clears it all the
same. `clear_mode_fault()` therefore reads CTLR1 before the STATR read
and writes the value it kept, the two steps adjacent, and clears the
flag on both; the RCC reset pulse puts it down on both as well (the
findings below).

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

One interrupt a block: every frame that comes back was clocked out
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
(a misaligned one is refused: the controller would round its address
down in silence, 11.3.6). And the requests are raised after
both channels are programmed, in one store: a one-frame transaction can
complete before the launch returns, and a request raised after that
completion would stand into the next transaction, whose transmit block
would then complete with nothing clocked and whose receive block would
never start (measured, [dma.md](dma.md)).

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

`SpiHost<n, pins, TxEngine, RxEngine>` is the engine
[util/spi_bus.hpp](../../brio/util/spi_bus.hpp)'s `SpiBus` drives, and
its `Request` carries the fields every family's host shares
([design/spi-bus.md, "Realizations"](../design/spi-bus.md#realizations)),
with the rate and the frame size as this family's `SpiClock` and
`SpiDataSize`: a chip select the engine asserts in `start()`, which the
bus AO calls, a display D/C line, a CS setup time, a command phase and a data
phase with optional out and in buffers, the mode/rate/frame size per
request, and an `engine` field naming the engine that runs it. The
Request is laid out with no padding - the two `PinRef`s, the three
buffers and the reply in eight words, the lengths and the four settings
in two, 40 bytes - and it is lent for the call: a request that completes
inside `start()` is read where it lies and nothing of it is copied; one
that goes to the engines has what its tenure (the transaction's time on
the bus, from `start()` to its completion) needs after the return - the
select and the D/C, the two pads its completion raises - stored into the
engine's own Tenure in four word stores, and the reply stays the arbiter's.

**The one busy check comes first** ([design/spi-bus.md, "One busy check,
before anything moves"](../design/spi-bus.md)): on a host with engines,
`start()` asks whether a block of its own is still on them, and if so
answers true with `spi_rejected`, whichever engine the request names,
before the select, the D/C or the block moves. The block in flight goes on
to its end and its own answer. The arbiter never starts a request over one
in flight, so only a program that drives the host directly meets it. No
other owner can hold these channels ([dma.md](dma.md), "One owner a
channel"), and an engineless host has nothing in flight to ask about.

**The two engines, and the request names one** ([design/spi-bus.md, "Two
ways to run a request"](../design/spi-bus.md)). The host chooses nothing
and reads no cost at run time; `start()` tests `r.engine` once, the
request's fields read where they lie, and a polled request pays that test
and, on a host with engines, the load and branch of the busy check and of
the owed reset's test.

- **Polled** (`SpiEngine::polled`): the loops below inside `start()`, at every
  length and on every host, whoever called `start()` spinning for the whole
  transaction under one spin budget and the engines never touched. Through
  the arbiter that is its dispatch: the client that posted does not wait and
  its buffers stay lent until `SpiDone`, and under `Tenuto` every other active
  object waits for the loop's whole wire time. A polled request needs no
  engines and no vector and runs with interrupts masked as it does with them
  enabled.
- **DMA** (`SpiEngine::dma`): the two channels at every length, one frame
  included, `start()` returning false and the receive channel's vector
  ending the transaction. It needs engines bound and their two vectors
  bound. In this order:
  1. on a host with no engines it is refused: `start()` answers true with
     `spi_rejected` in `status()`, before it touches the block, the select or
     the D/C, and `has_dma` says so at compile time;
  2. a request with nothing to move (no command frame and no data frame)
     completes on the spot, `spi_ok`;
  3. what the engines cannot carry is refused the same way, nothing on the
     wire: 16-bit frames over engines of `uint8_t`, or from an odd address
     in either buffer (the frame is one 16-bit access, and the controller
     would round an odd address down in silence, 11.3.6); on the CH32V303's
     DMA1 - SPI1 and SPI2 - a buffer whose run crosses a 64 KB boundary
     (11.2.3's note (1), measured as a wrap, [dma.md](dma.md)).
     These are every refusal the channels' own launch could answer, asked
     before anything moves. A program whose buffers may sit anywhere names
     `SpiEngine::polled` for them, or aligns them;
  4. a command phase with a data phase behind it runs in the loop inside
     `start()`, D/C rising after its last frame's last edge, and the data
     phase on the engines: one fixed rule at any length, a command being
     short by nature and a block of its own costing a second launch and a
     second interrupt. A stall in that loop ends the request as a polled
     one: `start()` true, `spi_stalled`, the select released, the block
     reset. A command phase that is the whole request rides the engines as
     the request, D/C rising at its completion;
  5. everything else is one block on the engines, one interrupt.

  `start()` answers true for a polled request, an empty one and a refused
  one, and false only when the engines have it.

**When a request ends, the wire has ended**, on either engine and with or
without a select: `start()` answering true, or a vector body answering
true, comes after the last SCK edge, and the select rises after it too.
The polled write waits BSY's fall; the polled read's last frame and the
receive channel's completion rest on RXNE, which this silicon raises at the
frame's end in all four modes ("No FIFO" above). So the host adds no wait
for it, and letter s stands as the verdict ("Bench findings"). At every end,
ok or failed, the D/C and the select are high: a stalled command phase, on
either engine, raises the D/C and then the select before the block's reset,
a block that faults raises both in the channel's vector, and `recover()`
over a request a timed arbiter gave up on raises the select and then the D/C
(letter f).

The polled loops tell two shapes apart by the in buffer. A write (no in
buffer, and every command phase) writes ahead at every rate: the loop
writes on TXE and never reads the answers - the tail waits TXE and then
not BSY, and the DATAR-then-STATR read clears RXNE and OVR. A receive (an
in buffer) keeps one frame in flight at every rate: the loop pins the next
frame's load inside the wire time and puts the DATAR read and the next
write in adjacent instructions, so nothing is queued behind the frame
shifting and OVR cannot rise. A polled flag that never comes within the
transaction's one spin budget ends the transaction with `spi_stalled`, and
a request the engines cannot run is answered `spi_rejected` - the codes
every host spells from `util/spi_bus.hpp` beside the engines' own
`spi_dma_fault` (design/spi-bus.md).

**What each answer asks of the program.** The answer reaches the requester
as the `status` field of its `SpiDone`:

```cpp
[](const brio::SpiDone& d) {             // a lambda of MyAo's brio::match
    if (d.status != brio::spi_ok) { /* below; on any status the buffers are the program's again */ }
    return handled();
},
```

| `status` | what happened | what it leaves | who acts |
|---|---|---|---|
| `spi_rejected` | the arbiter's FIFO was full, or a request naming the engines that they cannot carry, or on a host with none, or any request while a block of the host's own is in flight | nothing on the wire | the program: a full FIFO, or a block in flight, may be tried again once it has ended; a refusal never as it stands - `static_assert(Bus::has_dma)`, aligned buffers, or `SpiEngine::polled` for that request |
| `spi_stalled` | a polled flag never came within the budget | D/C and select released, then the block reset and put back to the applied mode, rate and width, before `start()` returns | nobody: the next request starts on a clean block. A closed gate is the exception below |
| `spi_dma_fault` | a channel reported a transfer error | requests down, both channels put away, D/C and select released, a reset owed, which the next `start()` or `prime()` pays before anything else | nobody: the arbiter answers and goes on, and the next request starts on a clean block |
| `spi_timeout` | a timed arbiter's limit passed with no completion | the arbiter has already called `recover()`: select and D/C released, the engines put away, the block reset | the program judges the device |

**After `spi_stalled` or `spi_dma_fault` the program calls nothing**, in direct
use or under the arbiter: the host does it, because it exists in both and
because the arbiter goes on to the next waiting request before the requester
has run. A request queued behind the failed one, in the same dispatch, starts
on a clean block and runs exact (letter r).

After a stall the select goes first and the reset second, inside `start()`: the
reset takes CPOL down with CTLR1, so in modes 2 and 3 it moves the SCK pad, and
the device must have seen the transaction end. A stall holds the core for its
whole budget, every other active object waiting under `Tenuto`. The budget is
derived from the clocks, never sized by hand, and kept as one shift that
`init()`, `rebase()` and every applied change of rate or width work out: twice
the request's frames in core cycles (a frame is 8 or 16 bits of 2^(BR + 1) bus
cycles, the bus's cycle 1, 2, 4, 8 or 16 core cycles by its prescaler), plus 256,
counted in polls of STATR of at least a core cycle each. So it never runs out before the wire has had
twice its own time; and since a poll measures 9 to 12 cycles, it holds the core
for some twenty times the wire's time when nothing moves, 184 000 cycles (1.3 ms
at 144 MHz) for sixteen frames at /64 of SPI1 on the CH32V203C8T6 (letter f
below). The reset adds at most some 580 cycles to
that (letter f), and the `disable()` in it spends nothing on a stalled block: it
waits only when SPE reads back set, and every stall staged here reads it clear.

After a transfer error the receive channel's or the transmit channel's vector
decides and does not work: no disable, no reset, no configuration and no wait
there. Up to a frame is still leaving the block and one stands in the receive
buffer behind it, twelve runs of twelve (letter f); the reset the next `start()`
runs first clears them, so no frame of the faulted block comes back in front of
the next request. A program that frames the select by hand calls `prime()` before
it falls, as it does anyway, and `prime()` pays the reset there, with the select
high.

**The limit is a closed clock gate.** A reset does not open one, so the next
request stalls too, on its own budget, for as long as the gate stays closed, and
the reset leaves such a block alone. Measured: with the RCC pulse and its writes
run on a closed gate, a request on the gate opened stalled once more; with
neither, a gate closed and opened keeps the registers, and the same request runs
`spi_ok`. `recover()` answers false there.

**What the reset does not put back** is what the applied mode, rate and width do
not hold: a hardware NSS arrangement or the high-speed read a program set on the
resource is the program's to set again, which it knows to do from the status. A
request on the engines over a block that stops clocking in flight has no completion,
and a bus with no timeout then stays busy for ever: that is `spi_timeout`'s case,
and `recover()` is its way back, called by a timed arbiter and open to a program
with the bus idle. `recover()` under a block on the engines takes that request
away, its completion never coming, which is what `busy()` is for; it answers
false when a bounded wait ran out or the gate is closed.

`apply()` compares one word: the request's mode, rate code and width are
folded into CTLR1's own bits over the applied base (CPHA and CPOL are
`SpiMode`'s two bits in the register's places, BR the code, DFF the
width), compared with the word in force, and written - with the
disable/enable pair 20.4.1 demands - only on a change.
Nothing is flushed at `start()`: the chapter gives no way for a frame to
stand in the receive buffer at the start of a transaction but a tenure
that ended without reading what it clocked, which is a fault path here,
and every fault path has reset the block before the next request's first
frame.

Beside it: `has_dma` (= `has_engines`);
`init(clock, max_sck_hz)` with an optional
bus-wide SCK ceiling, `rebase()` for a dynamic clock's fan-out (the
ceiling and the chip-select setup's busy-wait follow the new bus),
`clock_for()` / `sck_hz()` / `reference_hz()`, `prime()` (mode and rate
now, moving no data - a CPOL flip is an edge, so prime before the select
falls; it pays a reset a transfer error left owing), `bit_order()` (a property of the wire and so a bus-level verb),
`pad_speed()` (the slew class of SCK and MOSI, a choice that survives
the next `init()` and a reset), `start()`, `dma_rx_isr()` and `dma_tx_isr()` (the
two channels' bodies, each reading its own flags; `dma_isr()` is both,
for a program that binds the two vectors to one function), `status()`,
`busy()` (a block on the engines), `recover()`, `release()` and
`claim_nss_pad()`. `init()` opens the gate and pulses the block's reset
first, and `release()` closes the gate with SPE cleared and no reset
pulse, as every transport of this family does ([dma.md](dma.md), "A
released requester"; this block's own requests do not outlive SPE, so no
stray waits for the next `init()` here).

### What each engine costs the core

The host decides nothing by cost; a program that names the engine may
want to know what it buys. On the engines a request costs the core their
fixed cost, E - `start()`, the launch of both channels, the receive
channel's one interrupt and the completion - whatever its length: the
wire's time is the core's own. Polled, it costs the core the whole
request: a fixed part F and each frame's time, the wire's frame or the
loop's own turn, the longer. So the engines cost the core fewer cycles
from the length N at which F + N x frame reaches E, N = ceil((E - F) /
frame), and below it the loop does. `test_vx03_spi` letter g measures every
figure through one call of the suite, so that the call's own cycles
cancel: E as an engined request of 32 byte frames at /4 less its frames'
time in the block (the frame there the difference of a 256- and a 32-frame
request over 224), for a read and for a write; at every code each loop's
frame as the difference of a 12- and a 4-frame polled line over 8, and F
as the 4-frame line less its frames, the line extrapolated to no frames;
and N from those, which it then walks at a read at /4 and /2 and a write
at /2 - the fewest frames whose polled line costs E - and finds equal to
the arithmetic. At 144 MHz, ten runs a part, cycles:

| figure | CH32V203C8T6 (V4B) | CH32V303VCT6 (V4F) |
|---|---|---|
| E, SPI1, a read / a write | 601 / 608 | 669 to 685 / 645 to 664 |
| F, SPI1, a read / a write (at /4) | 296 / 317 | 256 / 265 to 268 |
| E, SPI2, a read / a write | 551 / 557 to 561 | 596 to 611 / 572 to 582 |
| F, SPI2, a read / a write (at /4) | 259 to 261 / 291 to 293 | 225 / 225 to 229 |

N, the frames from which the engines cost the core fewer cycles, byte
frames, a read / a write (letter g measures byte frames alone; 16-bit
frames have no table here, and at /2 and /4, where the loop's turn and not
the wire sets the frame, halving the byte figure would not give theirs):

| code | SPI1, V4B | SPI1, V4F | SPI2, V4B | SPI2, V4F |
|---|---|---|---|---|
| /2 | 11 / 19 | 13 to 14 / 23 to 25 | 6 / 9 | 7 to 8 / 11 |
| /4 | 6 / 10 | 10 to 11 / 12 to 13 | 4 / 5 | 5 / 6 |
| /8 | 4 / 5 | 6 / 6 to 7 | 2 / 3 | 3 / 3 |
| /16 | 3 / 3 | 3 / 3 to 4 | 2 / 2 | 2 / 2 |
| /32 | 2 / 2 | 2 / 2 | 1 / 1 | 1 / 1 |
| /64, /128, /256 | 1 / 1 | 1 / 1 | 1 / 1 | 1 / 1 |

In words: on SPI1, whose bus is HCLK, a single frame costs the core more
polled than on the engines at /64 and slower on both cores, while at /2 a
read repays the engines from 11 frames on the CH32V203 and 13 or 14 on the
CH32V303 and a write from 19 and 23 to 25; on SPI2, whose bus is half of HCLK
and whose frames are twice as long in core cycles at a code, a single
frame at /32 and slower. The table is SPI1's and SPI2's; SPI3 shares
SPI2's bus and is not in it. The arithmetic is advice: nothing in the code
reads it, and a program that names the engine where the loop would have
been cheaper pays time, never correctness. Through the arbiter a request
on the engines also pays the post of `TransferDone` and its dispatch,
which E leaves out, so there N sits a little higher. The V4F's E is the
longer by its DMA vector, a naked trampoline over a handler that saves
the f-registers ([platform.md](platform.md)); its loops are the quicker by
a few cycles. A loop's figures move by a few cycles with where the image
places it (on the CH32V203C8T6 `bench_vx03`'s letter e reads the receive
loop's frame on SPI1 at /4 in 8-bit frames 10 cycles over the wire and
`test_vx03_spi`'s letter g 19; on the CH32V303VCT6 14 and 10 to 11).

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
(void)Bus::init(clock);                  // no ceiling, the default column: nothing to refuse

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
r.engine  = brio::SpiEngine::polled;
(void)Bus::start(r);                     // a polled request answers true, done or not: status() decides
if (Bus::status() != brio::spi_ok) { /* spi_stalled: id holds nothing */ }
```

A shared bus under the arbiter, the transactions on SPI2's two DMA
engines (DMA1's channel 5 transmits, 4 receives) - `Fast::init(clock)`
in `main()` as `Bus::init()` above, `r` the request above:

```cpp
using Fast = brio::SpiHost<2, brio::spi_default_pins<2>,
                           brio::DmaTxEngine<1, 5>, brio::DmaRxEngine<1, 4>>;
static_assert(Fast::has_dma);            // a request naming the engines wants them
using Arb = brio::SpiBus<Fast, Platform, 4>;
BRIO_CH32_VECTOR(dma1_channel4_handler) {
    if (Fast::dma_rx_isr()) { brio::post<Arb>(brio::TransferDone{Fast::status()}); }
}
BRIO_CH32_VECTOR(dma1_channel5_handler) {
    if (Fast::dma_tx_isr()) { brio::post<Arb>(brio::TransferDone{Fast::status()}); }
}
Fast::Request q{};                       // r's fields, then
q.engine = brio::SpiEngine::dma;         // the command in the loop, the data on the engines
q.reply  = brio::reply_to<MyAo, brio::SpiDone>();
brio::post<Arb>(q);
```

The engines with no arbiter, on the two slots the tables wire to the
instance - DMA1 for SPI1 and SPI2, DMA2 for the CH32V303's SPI3 - and a
request of one frame on them, which they run as they run any length:

```cpp
using One = brio::SpiHost<1, brio::spi_default_pins<1>,
                          brio::DmaTxEngine<1, 3>, brio::DmaRxEngine<1, 2>>;
(void)One::init(clock);                  // binds both engines to DATAR; nothing to refuse
volatile bool one_done = false;
BRIO_CH32_VECTOR(dma1_channel2_handler) {
    if (One::dma_rx_isr()) { one_done = true; }   // One::status() says how it ended
}
BRIO_CH32_VECTOR(dma1_channel3_handler) {
    if (One::dma_tx_isr()) { one_done = true; }   // a transfer error
}
static const uint8_t frame[1] = {0x5A};
One::Request w{};
w.tx     = brio::lend<brio::Lease::reply>(frame);
w.len    = 1;
w.clock  = brio::SpiClock::div4;
w.engine = brio::SpiEngine::dma;
if (One::start(w)) { /* true: refused, One::status() is spi_rejected */ }

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
  block on the engines, neither buffer named, the best of three each - is
  timed on the
  core's counter, which counts HCLK, in thousandths of its SCK periods at
  the instance's own bus over the division. SPI1 read 1000 at /4, /16 and
  /64 and in 16-bit frames at /4, SPI2 1000 at /2 and /16 (999 in some
  of twenty runs): the clock does not pause between frames while the
  engine refills (20.2.2), and each bus divides as the clock tree says.
  The polled path is never faster than its wire at the rate asked - SPI1
  on its second column, 64 frames: 1220 cycles at /2 against the wire's
  1024, 2247 to 2273 at /4 against 2048, 131266 at /256 against 131072 - nor is letter c's polled ladder on the
  strap, one frame in flight at every code (30 cycles a frame at /2
  against 16, 52 at /4 against 32, 2065 at /256 against 2048).
- **THE HOST ABOVE THE WIRE**, `bench_vx03`'s letter e on SPI1 with no
  engines, MISO floating, each line the best of 4 from a fresh tick (the
  wire's cycles a frame: 32 and 128 for 8-bit frames at /4 and /16, 64
  and 256 for 16-bit ones). A POLLED WRITE of 256 frames runs at the
  wire: 32, 128, 64 and 256 cycles a frame; the vendor's own loop on
  the same board (the EVT's 2Lines_FullDuplex shape, transcribed into a
  scratch program on brio's crt and console) reads 39, 128, 64 and 256,
  and at /4 in 8-bit frames LOSES A FRAME IN TWO RUNS OF FOUR with
  nothing live but the tick: a loop that refills on TXE whenever it can
  falls behind a 32-cycle frame (its turn is 39) and the receive side
  overruns. A POLLED RECEIVE, one frame in flight at every rate, is
  42 and 138 cycles a frame in 8-bit frames at /4 and /16 (1.31 and 1.08 times the wire's frame, larger being slower), 84 and 276 in 16-bit ones (1.31 and 1.08). No line takes an
  interrupt. What the turn costs at every rate is the letter's `spi.frame`
  table, HCLK cycles a frame over the wire from 256 less 16 frames, the
  least of five runs with interrupts masked, SCK and MOSI at the medium
  class, on both parts:

  | over the wire, 144 MHz | /2 | /4 | /8 | /16 | /32 | /64 | /128 | /256 |
  |---|---|---|---|---|---|---|---|---|
  | CH32V203C8T6, SPI1, 8-bit, mode 0 | 14 | 10 | 14 | 10 | 14 | 10 | 14 | 10 |
  | CH32V203C8T6, SPI1, 8-bit, mode 1 | 14 | 22 | 14 | 22 | 14 | 22 | 14 | 22 |
  | CH32V203C8T6, SPI1, 16-bit, both modes | 16 | 20 | 16 | 20 | 16 | 20 | 16 | 20 |
  | CH32V203C8T6, SPI2, 8-bit, both modes | 20 | 30 | 22 | 20 | 30 | 22 | 20 | 30 |
  | CH32V203C8T6, SPI2, 16-bit, both modes | 24 | 30 | 28 | 24 | 30 | 28 | 24 | 30 |
  | CH32V303VCT6, SPI1, 8-bit, both modes | 18 | 11 | 18 | 17 | 15 | 20 | 21 | 14 |
  | CH32V303VCT6, SPI1, 16-bit, both modes | 17 | 15 | 14 | 21 | 17 | 18 | 20 | 15 |
  | CH32V303VCT6, SPI2, 8-bit, mode 0 | 26 | 20 | 28 | 20 | 28 | 20 | 28 | 20 |
  | CH32V303VCT6, SPI2, 8-bit, mode 1 | 26 | 32 | 28 | 32 | 28 | 32 | 28 | 32 |
  | CH32V303VCT6, SPI2, 16-bit, both modes | 28 | 24 | 28 | 24 | 28 | 24 | 28 | 24 |

  The wire's frame is the bits x the division x HCLK over the bus: 32
  cycles at /4 on SPI1 in 8-bit frames, four times that on SPI2 in 16-bit
  ones. RXNE rises at the frame's end ("No FIFO" above), so the turn shows
  at every rate and in both modes, no half bit hiding it. The figures step
  by a few cycles from code to code in no order of the rate, the same to a
  tenth of a cycle in the letter's three runs: the frame's length against
  the five instructions of the poll decides where in a poll RXNE lands,
  which is a reading of the pattern and has not been bisected. Placement
  moves them too: on the CH32V203C8T6 the same loop on SPI1 at /4 in 8-bit
  frames reads 10 cycles here and 19 in `test_vx03_spi`'s image (letter g). A POLLED REQUEST OF THREE BYTES - a
  command and two data, the D/C scripted, the select and the D/C on two
  pads - costs 232 cycles above its wire time at /16 (the stopwatch's 28
  inside), one byte 184, sixteen 224: the price of a DCS command. Of the four pin
  edges each is one BSHR or BCR store inline through `PinRef::set/clear`
  (pin.hpp forces the two verbs inline; as calls they cost the 3-byte
  request 25 cycles more, measured). On the CH32V303VCT6 - the V4F, the
  image built under ilp32f, the same letter on the evaluation board's
  SPI1 pads with MISO undriven - the shapes land where the
  CH32V203C8T6's do: the polled write of 256 frames at 32, 128, 64 and
  256 cycles a frame; the polled receive at 46 and 150 cycles a frame in 8-bit frames (1.44 and 1.17 times the wire's frame), 86 and 276 in 16-bit ones (1.34 and 1.08); the polled request of three
  bytes 213 cycles above its wire time at /16, one byte 171, sixteen
  210; every line ended with status 0 and no overrun standing.
- **THE BEAT IS THE FRAME, on the strap** (letter c, ten runs): sixteen
  16-bit frames over half-word aligned buffers ride both engines as
  half-words - byte-exact through the loop, ONE channel interrupt for the
  transaction and no SPI one; the same frames over buffers one byte off,
  which the engines cannot carry, are refused, `spi_rejected`, no
  interrupt at all and the in buffer untouched, and named polled they run
  in the loop, byte-exact.
- **A COMMAND PHASE IN THE LOOP, THE DATA ON THE ENGINES** (letter c, on
  the strap, ten runs): a command of one frame and one of 32 frames at /8
  each run in the loop inside `start()`, and the 32 data frames after
  either ride the engines, one channel interrupt, the data exact through
  the loop.
- **THE ENGINE A REQUEST NAMES** (letter g, no wire, both parts, thirteen
  runs each): a dma request on the engineless host answered at once
  `spi_rejected`, its select's output latch untouched, CTLR1 not
  rewritten though the request asked for another mode, rate and width,
  the block idle - and the same request polled `spi_ok`, its select
  framing it. On the engined host ONE FRAME named dma at /2, /4 and /16 on
  SPI1, read at both levels of a port-held MISO and written: `start()`
  false, one channel interrupt, `spi_ok`, the frame read exact, nine of
  nine - at codes where the loop costs the core fewer cycles ("What each
  engine costs the core" above).
  THE REFUSALS, each from a block primed at mode 0 and /64 with the request
  at mode 3 and /8: 16-bit frames with the out buffer odd, with the in
  buffer odd, a command alone at an odd address, 16-bit frames over engines
  of `uint8_t`, and on the CH32V303's
  DMA1 a byte run across 0x0801 0000 in flash, a run one byte past it, a
  half-word run one half-word past it and an in buffer running past the
  SRAM's 64 KB page - each `spi_rejected` with `start()` true, the select's
  and D/C's latches unmoved, CTLR1 as primed, no frame clocked, no request
  raised and no channel interrupt, and the next read on the engines exact
  after each. Runs ending at 0x0800 FFFF and starting at 0x0801 0000, bytes
  and half-words, ride DMA1 `spi_ok`, and the byte run across the boundary
  rides DMA2 on SPI3 `spi_ok` (note (3)). A polled read of 256 frames at /4
  and at /64 runs the loop on the engined host, no channel interrupt, and
  the same read at /4 with global interrupts masked, `spi_ok`: a polled
  request needs no vector.
- **THE COMMAND RULE, D/C'S EDGE AGAINST SCK'S** (letter g, both parts):
  SPI1 on its second column, TIM2 counting SCK's rising edges on PB3 and
  its channel 1 capturing that count at D/C's edge on PA15, in mode 0. A
  command of 2 frames then 4 of data, named dma at /64 and at /8: one
  channel interrupt, D/C rising at exactly the 16th edge of 48 and falling
  before the first, the same as the polled run of that request; a 16-bit
  command at an odd address before aligned data, D/C at the 16th edge of
  80, the data on the engines; a command alone of 4 frames on the engines,
  one interrupt, D/C rising at the 32nd edge of 32, as polled.
- **WHEN A REQUEST ENDS THE WIRE HAS ENDED** (letter s, no wire, both parts,
  thirteen runs each). The SCK pad's input read on the first instruction
  after the end - after `start()` returned true, or in the vector on the line
  after the body answered true - in the four modes at /256 and /64, a read, a
  write and a command alone, polled and on the engines, through a select and
  through a null one, four times each: 192 of 192 at the mode's idle level on
  each engine and each instance (SPI1, SPI2, and SPI3 on the CH32V303VCT6),
  the select's latch high. What the reading can see: a request that ended
  half a bit early would leave SCK away from idle for that half bit in modes
  0 and 2, 128 cycles on SPI1 at /256 and 256 on a PB1 instance, 32 and 64 at
  /64. The polled path reads the pad a few instructions after its last flag;
  the vector reads it 185 to 190 cycles after SCK's last edge on SPI1 and
  166 to 180 on SPI2 and SPI3, both cores (measured: the thread sampling the
  pad until the vector's stamp), so on a PB1 instance at /256 an early end
  would show and on SPI1 it would not. RXNE itself, sampled
  with the pad in a masked loop of 31 to 39 cycles a turn on the resource, came
  up after the last sample away from idle in every mode at both rates on every
  instance, ten runs on each part: one turn after it in most cases, two or three
  on the CH32V203C8T6's SPI1 at /64 (71 cycles in modes 0 and 2, 109 in modes 1
  and 3, every run) and in a few runs of the CH32V303VCT6's instances (66 to 73,
  and once 118), BSY last seen up on that sample or a turn or two after it. The
  flag never leads the pad's return to idle, where an early end would show it
  up to half a bit, 256 cycles, ahead.
- **A POLLED WAIT THAT RUNS OUT ends `spi_stalled`** (letter f, both
  parts): a polled write of 16 frames started with SPI1's gate closed
  under the engine - every STATR load reading zero, so TXE never comes -
  returned `spi_stalled` after 183949 to 183975 cycles on the CH32V203C8T6
  and 167268 to 167305 on the CH32V303VCT6, over the budget's 16640
  spins (16 frames of 8 bits at /64 of PB2, twice over, and 256): 11 and 10
  cycles a spin of a loop of five
  instructions, and 12 and 9 in other layouts of the same loop (the STATR load, the bit test, the budget's decrement and
  test, the branch - poll_write's in the release listing), so the bound
  the suite judges, 5 to 20 cycles a spin, holds on both. The next request
  on the closed gate stalled too, on its own budget; with the gate open the
  same request ran `spi_ok`. Ten runs a part.
- **A FAILED REQUEST ENDS WITH THE D/C AND THE SELECT HIGH** (letter f, both
  parts, ten runs a part): a command phase stalled on the polled engine and in
  front of a data block on the engines, each `spi_stalled` inside `start()`,
  and a data block faulted behind its command phase, `spi_dma_fault` in the
  channel's vector; and `recover()` over a command alone in flight, both
  latches read low under the block and high after. The select and the D/C are
  read by their output latches, cleared before the request, so each set latch
  is the host's own store. The control, `recover()` with its D/C store taken
  out, fails that verdict in three runs a part.
- **THE ONE BUSY CHECK** (letter g, both parts, ten runs a part): sixteen frames
  at /256 on the engines, 32768 cycles of wire, and behind them a request naming
  the engines and one naming the loop, in mode 3 and at /8: each `spi_rejected`
  with `start()` true, the select's and D/C's latches and CTLR1 unmoved, the
  block in flight going on to its own `spi_ok`, and the next read exact.
- **THE NEXT REQUEST WITH NOTHING CALLED, AFTER A STALL** (letter f, no
  wire, ten runs a part, and letter r, the far client's judgement, three
  runs a part). The stage is one a reset cures: the block's enable taken down under
  the host with the request's own word primed, so `start()` writes nothing on
  the way in (a changed word's disable and enable would raise SPE and cure it).
  On letter f, two frames left standing with RXNE and OVR up and MISO held high
  by its port; then a polled write of 16 frames, a polled read of 16, or a
  command phase run polled before a data block, MISO held low. Every one ended
  `spi_stalled` in 6341 to 201277 cycles on the CH32V203C8T6 and 5381 to
  167876 on the CH32V303VCT6, by the frames' budget, the select's latch high, and on `start()`'s
  return CTLR1 held the applied word with SPE up in modes 0, 2 and 3, STATR read
  TXE alone and SCK stood at the mode's idle level. The next polled read and the
  next read on the engines each read all zeros: no frame left
  standing came back in front. On letter r, with a far client armed: a stall
  in each of the four modes and in a command phase, each followed by a polled
  and by an engined window of 32 frames at /64 (/8 behind the command phase), both
  ends exact.
  The reset itself, `recover()`'s whole call on an idle block, is 575 cycles
  on the CH32V203C8T6 and 551 to 554 on the CH32V303VCT6.
- **A REAL TRANSFER ERROR, AND THE NEXT REQUEST** (letter f ten runs a part,
  letter r three). A channel is refused at the top of the SRAM region, 0x2FFF F000,
  read or written ([dma.md](dma.md)), and on a read past the memory it can reach:
  a block whose out buffer ends at the last byte below 0x2000 5000 on the
  CH32V203C8T6, or below 0x0807 8000 on the CH32V303VCT6, runs 16 frames and is
  refused mid-block. No address faulted the core, and neither board wedged. Each
  of the three - the mid-block one, the transmit channel's first beat and the
  receive channel's first - in modes 0 and 3: the vector's body answered true,
  `spi_dma_fault`, the select's latch high, both requests down and the enable
  standing, nothing reset in the vector; twelve runs of twelve then found a frame
  standing in the receive buffer. MISO moved from high to low after the fault, the
  next polled read and the next engined read each read all zeros, on SPI1 on DMA1.
  On letter r, on the pair's instance, the same three, then a window exact at both
  ends, twelve windows of twelve on SPI2 (DMA1) on the CH32V203C8T6 and SPI3 (DMA2)
  on the CH32V303VCT6.
- **QUEUED BEHIND A FAILURE THROUGH `SpiBus`** (letter r, three runs a part):
  one client posting two requests from one dispatch, the first staged to stall
  or to fault mid-block, the second a window with its select in the Request. The
  replies came in order, `spi_stalled` or `spi_dma_fault` and then `spi_ok`, the
  second window exact at both ends, polled and on the engines, modes 0 and 3, four
  of four each way. Nothing called `recover()`.
- **THE TRANSMIT INTERRUPT ON A HOST** (letter f): SPI1 bare, no pad
  handed over, a vector writing the next frame on each TXE and silencing
  TXEIE after the last - 64 frames at /64 in 65 entries, every frame out
  in order by the CRC unit's own sum (TCRCR 0x1E, the bitwise loop's),
  back to back: 32895 to 32897 cycles on the CH32V203C8T6 and 32887 to
  32902 on the CH32V303VCT6 against the frames' 32768, thirteen runs a part.
  `txe_interrupt()` and the resource's `isr()` reporting TXE under its
  enable are what it exercises; the host engine arms no interrupt of the
  block (the class comment), this being the shape of a transmit-only
  stream with no reader, for a program that drives the resource itself.
- **THE WRITE-AHEAD ON THE RESOURCE, AND THE HOST UNDER THE SAME LOAD**:
  the vendor's write-ahead loop run on the resource at every code and
  width, 1024 frames a run, eight runs a point under a console print in
  flight, loses a frame in seven or eight runs of eight at 32, 64 and 128
  cycles a frame on both parts (four to seven at 16, where a run is too
  short to meet every interrupt) and in none at 256 and above; the host
  under the same load, both shapes of the polled loop, every code and
  width, finishes all sixty-four points with no overrun standing and
  `spi_ok`. ON A PB1 INSTANCE THE RESOURCE'S BOUNDARY MOVES ONE CODE
  (`spi2.ahead`, `spi2.live`, SPI2 at 72 MHz of bus under 144 of HCLK, both
  parts): five to eight runs of eight at 32, 64 and 128 cycles a frame - /2,
  /4 and /8 in 8-bit frames, /2 and /4 in 16-bit ones - and none at 256 and
  above; the host's sixty-four points finish the same way.
- **THE ENGINED TRANSACTION'S FIXED COST**, `bench_vx03`'s letter d, a
  write on SPI1's engines with MISO floating at 144 MHz: 503 cycles above
  the frames' own wire time at /4, 498 at /16, ONE interrupt (the receive
  channel's) - the launch 320 of it (the refusals' tests inside), the
  completion's handler 142 with its stamps ([dma.md](dma.md)). 256 frames
  at /4 are 8695 cycles against 8192 of SCK, at /16 33266 against 32768.
  What that fixed cost weighs against a polled transfer is "What each
  engine costs the core" above. On the CH32V303VCT6: 577 to 580 cycles
  above the wire at /4 and 558 at /16, ONE interrupt - the launch 395 to
  398, the 64 KB test of both buffers inside, the completion's handler 142
  to 147 with its stamps - and 256 frames at /4 in 8769 cycles against
  8192, at /16 in 33326 against 32768. One run a part.
- **16-bit frames through the engines**: a 256-frame write of half-words
  in 16897 cycles at /4 against 16384 of SCK (1.03 times the wire), and
  66048 at /16 against 65536, one interrupt each. The CH32V303VCT6: 16949
  cycles at /4 (1.03) and 66099 at /16, one interrupt each.
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
- **THE STRAP IS CLEAN TO 36 MHz AND BREAKS AT 72.** Every BR code at /4
  and slower carried 256 bytes byte-exact; /2 did not. Four kilobytes at
  /4 (36 MHz of SCK), sent as sixteen chunks, came back byte-exact.
- **The polled read on the strap takes no interrupt**: 32 frames
  byte-exact, SPI1's vector never entered; the four modes, both widths,
  both bit orders and the CRC are judged by a foreign client (the pair's
  section).
- **A host clocks itself.** With MOSI released and MISO held by the
  port, every frame comes back all ones or all zeros with the level, at
  both widths, both bit orders and all four modes - the wireless proof
  that the receive path is the pad and not a leftover.
- **The mode fault takes the role away, and the chapter's way back
  wants nothing between its two steps** (letter e, twelve runs on each
  part, every attempt from a fresh fault with interrupts masked).
  Clearing SSI on a software-managed host raises MODF and the block
  loses MSTR and SPE with it (CTLR1 0x228, STATR 0x22 at the fault), and
  the error reaches the instance's vector under ERRIE - one entry
  carrying the mode-fault bit. 20.2.7's "a read or write operation to
  STATR, then a write to CTLR1" clears the flag on the CH32V203C8T6 and
  on the CH32V303CBT6 with nothing between the STATR read and the CTLR1
  write, and a write raising SPE takes after it; a second STATR read
  between the two changes nothing. A read of CTLR1 or
  of CTLR2 between them leaves MODF standing on the CH32V203C8T6, every
  time, and on the CH32V303CBT6 the flag clears all the same. The
  manual says nothing of the order. The shape WCH's own library
  documents (`SPI_I2S_GetFlagStatus` then `SPI_Cmd`, a read-modify-write
  of CTLR1) has that read in it; `clear_mode_fault()` reads CTLR1 before
  the STATR read and writes the value it kept, and clears the flag on
  both parts (letter e judges it). While MODF
  stands, a CTLR1 write raising SPE does not take, so a demoted host
  cannot be re-enabled until the flag is down. Since MODF has taken SPE
  and MSTR with it, a demoted host must be reconfigured anyway, and the
  block's RCC reset pulse, which puts the flag down on both parts, is
  the first step of that.
- **CRCERR is the received CRC frame against RCRCR** (letter e, on the
  resource with no wire, MISO held by its port, one data frame and then
  CRCNEXT): with MISO held low the CRC frame read 0x00, which is the
  receive CRC of a zero frame, and no error; held high, the frame read
  0xFF against an RCRCR of 0xF3, the CRC of the one 0xFF frame received
  under the polynomial 7, and CRCERR rose. A zero written into its bit with ones elsewhere put it
  down, and a one written into it left it down.
- **The one-wire line mode turns around on the same pad**: BIDIMODE with
  BIDIOE drives a frame out of MOSI, and clearing BIDIOE makes that same
  pad the receiver, with the clock running on either side of the turn.
- **The overrun is a level with a sequence, and 20.2.7's sequence holds**
  (letter v, both parts, ten runs each, on the resource with no wire): two
  frames at /32 with nothing read leave STATR at 0x43, RXNE and OVR with TXE.
  DATAR then STATR clears it, with a load before the DATAR or none: that STATR
  load, a microsecond after the read, reads 0x42, RXNE down and OVR still up,
  and the load after it 0x02. A zero written to the flag leaves 0x43.
  `clear_overrun()` and `flush_rx()` each leave 0x02 from both standing, and so
  does the tail of a polled write of sixteen frames at /8, whose answers overrun
  the buffer from the second frame on.
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

`test_vx03_spi`'s second set (`y`, letters d, j, k, l, m, n, o, q, r, t) on each board
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
judged at /4 and slower (a half period holds the 20 ns SCK edge the tables
allow into 30 pF), this end's read-back at /8 and slower (the client's MISO
valid 25 ns after its edge and the host's 5 ns of setup inside the half
period), the far POLLED run at /16 and slower for 8-bit frames and at /8 and
slower for 16-bit ones (two of its turns inside a frame). Ten runs of letters d, j, k, l
and m each way, every verdict green in all twenty; ten runs of n and o
each way, green; `y` whole at 51 verdicts on the CH32V203C8T6's third
image against the CH32V303CBT6's spi_peer, green in ten runs (the other
direction was not run with today's suite).

- **THE ENGINE A REQUEST NAMES, judged by the far client** (letter t, both
  ways, thirteen runs each): one frame named dma at /8, /16 and /32 of PB1,
  read and written, `start()` false, the far client hearing the frame and
  this end reading its answer, six of six; 16-bit frames from an odd
  address started inside the select's window, the far client armed and
  selected, refused `spi_rejected` in all four modes with the far client
  counting nothing of it, and the 32-frame window after it exact at both
  ends; a command of four frames in the loop then 28 data frames on the
  engines, in the four modes and both widths, every frame heard in order at
  the far client and the data's answers read exact, eight of eight; and a
  command alone of 32 frames on the engines, heard in order in the four
  modes.

- **THE LADDER, both ways, both host shapes**: the polled request and
  the engines (a request naming them, 64 frames) against the far client's
  DMA serve, and the polled request against its polled run, 64 frames a
  row - exact both ways at
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
  wire 8 x division x 2; from the `start()` call to the completion, the
  engines' fixed cost spread over the 64 frames): the polled request
  4128 on the CH32V203C8T6 and 4118 on the CH32V303VCT6 at /256 (the wire's 4096), 278 at /16 (256), 152 and 158 at /8 (128), 96 and 86 at /4 (64); the engines 263 to 264 at /16, 135 to 136 at /8 and 71 to 72 at
  /4 -
  the polled request at the wire plus its turn, the engines within their
  fixed cost of it at every code.
- **A HANDLER LONGER THAN A FRAME IS A GAP ON THE BUS** (letter q, both
  ways, fifteen runs each): TIM3's vector holding the core for some 340 to 380
  cycles every 720 (5 us) beside the polled receive loop at /8 of PB1, 128
  cycles a frame, 64 frames against the far client's DMA serve: ten windows
  of ten exact both ways, `spi_ok`, the peer's count right, OVR never
  standing. The window lengthens from 9770 to 15368 to 15502 cycles on the
  CH32V203C8T6 and from 10131 to 10136 to 15370 to 15514 on the CH32V303VCT6, by what the
  letter's arithmetic allows (2665 to 11800 and 2870 to 12212 cycles, the handler's
  length counted in its listing).
- **The extended exchange against a foreign unit**: 16-bit frames exact
  both ways on every shape and both far serves (report_extended); the CRC
  at both widths - CRC-8 (0x07) TCRCR 0x62 and RCRCR 0x5B, CRC-16
  (0x8005) 0x5FB6 and 0x2B13, each what a bitwise loop computes over what
  went and what came, the CRC frame read the far unit's own, CRCERR down
  at both ends, and a polynomial mismatch (0x07 here, 0x31 there) raising
  CRCERR at BOTH ends; and this end's WRITE shape - no in buffer, the
  transmit-only loop writing ahead - heard exact by a far client that only listens, polled
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
  the mode-fault bit, and `clear_mode_fault()` - 20.2.7's STATR read and
  CTLR1 write with nothing between them - putting it down (the
  CH32V203C8T6 as that host, the CH32V303CBT6's spi_peer pulling the
  select).
- **SSOE'S SELECT FALLS WITH THE FIRST FRAME, NOT WITH SPE**: a host with
  SSOE set keeps its NSS pad high under SPE until the first frame is
  written, drives it low from then to the disable, and the far client it
  frames carries sixteen frames exact both ways.
- **`recover()` AFTER A TRANSACTION A SECOND DEVICE KILLED** (letter n):
  an engined write of 2000 frames at /256 (57 ms) under a TIMED SpiBus (a
  per-bus timeout of 120 ms), this host's NSS pad made the multi-host
  input under the engine, and the far chip's ss_pulse taking the select
  low 30 ms after its ack: MODF 24 to 25 ms into the transaction - SPE
  and MSTR gone, so no frame came back for the receive channel to count
  and no completion was posted - the arbiter's answer `spi_timeout` exactly 120 ms after the
  post, and after `recover()` the block a software-selected host again,
  enabled, MODF down (the reset pulse is the recovery's), the same
  arbiter carrying the next transaction `spi_ok` and the command channel
  answering. Ten runs each way.
- **`rebase()` UNDER A DYNAMICCLOCK** (letter o): the pair's host a user
  of a DynamicClock of 144, 96 and 48 MHz on the HSI's PLL, a ceiling of
  6 MHz declared at init(), each switch's fan-out leaving the engine's
  bus at 72, 48 and 48 MHz and the ceiling at /16, /8 and /8 (4.5, 6 and
  6 MHz) - each what the new bus makes it; and at every rate a request at
  /2 ran clamped to the
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

- **The receive loop's frame in modes 2 and 3, and on SPI3**: the letter's
  table measures modes 0 and 1 on SPI1 and SPI2; modes 2 and 3 as more rows
  of the same table would measure them. SPI3 sits on PB1 as SPI2 does, and
  its row wants the evaluation board's jumpers free; the pair's ladder reads
  86 cycles a frame at /4 on it, in mode 0 and 8-bit frames.
- **What each engine costs the core on SPI3**: letter g measures SPI1 and
  SPI2, and SPI3 shares SPI2's bus; the same measurement run on SPI3's host
  on the CH32V303, the pair's wires idle, would put its row in the table.
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
- **The high-speed read across a reset**: HSCR reads zero with HSRXEN just
  written on both dies here, the bit write-only on them, so whether the RCC
  pulse takes the mode down is the manual's reset value and not a reading.
  Letter h's read at 36 MHz run after a stall would measure it.
- **`Spi<n>::disable()`'s bounded waits inside the reset**: no stage here reaches
  them, every stalled block reading SPE clear, and after a fault they wait out
  at most the frame leaving the block. A stall with SPE reading set would
  measure them, 400 000 polls at the outside.
- **`rebase()`'s cs_setup timing**: the delay rate it recomputes is
  written and compiled, and letter o's requests carry no setup time; a
  request with `cs_setup_us` timed from the select's fall to the first
  edge at two rates would measure it.
