# SERCOM SPI (SAM C21)

This document is for a programmer who needs an SPI bus on the SAM C21:
a host talking to one or more devices (a sensor, a flash chip, a
display) through the bus arbiter, or this chip answering as a client
to another board's host. `samc21/spi.hpp` is the SERCOM's SPI mode in
three parts. `Spi<n>` is the resource, one function per register fact
of one SERCOM instance. `SpiHost<n, pads, ...>` is the transfer engine
`util/spi_bus.hpp` drives, a task (in brio, a driver layer with state
of its own over a resource, served from its peripheral's interrupt or
called by an active object - this one called) whose Request carries the
same field names on every target, so a device client reads unchanged on
any of them. Each request names how it runs
([design/spi-bus.md](../design/spi-bus.md), "Two ways to run a
request"): `SpiEngine::polled`, the loop, inside the call that starts it;
or `SpiEngine::dma`, the DMA engines the board file names in the host's
template arguments ([dmac.md](dmac.md), its budget of two channels),
the call returning with the wire running and an interrupt ending it -
but for a refusal and a command loop that stalls, which end inside the
call ("Types and verbs").
`SpiClient<n, pads>` is the client role. The
arbiter, `SpiBus` ([design/spi-bus.md](../design/spi-bus.md) is the
model), is an active object (a component of a brio program with its own
event queue, served one event at a time by the kernel,
[docs/learn/introduction.md](../learn/introduction.md)) that owns the
bus on behalf of every other active object: a client posts one Request
describing one whole transaction and gets one `SpiDone` reply when it
is over, and the arbiter runs the transactions one at a time, in the
order they arrived.

The smallest use, a device client on the arbitrated bus, is the first
block of "How to use" below:

```cpp
using SpiHw = brio::SpiHost<1, my_pads>;
using SpiBus = brio::SpiBus<SpiHw, P>;
// in main, after the clock task's init(): if SpiHw::init(clock) answers
// false there is no bus; then run the kernel with SpiBus in its list.
// From an AO: take the rate from SpiHw::baud_for(6'000'000), an optional
// checked before use, fill a Request (cs, buffers as lend<Lease::reply>(...),
// baud, mode, engine = SpiEngine::polled), post it to SpiBus, and consume
// the SpiDone reply.
```

Line by line: `SpiHw` names the engine over SERCOM1 with the pads
`my_pads` (an `SpiPads` constant naming the four SERCOM pads and the
pins they are bonded to, "Types and verbs" below), every other template
argument at its default - generator 0, the SERCOM's core clock at the
CPU rate, and `InFlash`, no path placed in SRAM; `SpiBus` is the arbiter
over that engine on the platform `P` ([platform.md](platform.md)), with
`util/spi_bus.hpp`'s defaults of four pending requests and no timeout;
`SpiHw::init(clock)` takes the clock task's tag ([clock.md](clock.md))
and brings the instance up as a host (with no ceiling stated, a false
answer means a synchronization of the SERCOM never completed), and the
rest of `main()` starts the kernel - `brio::Tenuto<P, ...>::run()` with
`SpiBus` in its list of active objects, which calls every active
object's `init()` and never returns; and an active object (AO) fills a
`Request` - the chip select as a `PinRef`, the buffers lent with
`lend<Lease::reply>(...)`, the BAUD register value `baud_for()` computes
for the device's 6 MHz (an optional: empty when the rate is above half
the core clock or below what the eight-bit BAUD can divide down to,
93.75 kHz at 48 MHz), the mode,
`SpiEngine::polled` - posts it to `SpiBus` and receives `SpiDone` with the
status (`spi_ok`, `spi_stalled` when the request's spin budget ran
out, or `spi_rejected` for a request naming `SpiEngine::dma`, which this
host, with no engines named, cannot run). No vector is bound: a polled
request needs none, and a host with no engines arms no interrupt.

What decides correctness even there. `init()` comes after the clock
task's, as its comment asks: the BAUD values `baud_for()` answers, the
optional SCK ceiling and the chip-select delay are computed from the
rate the clock type claims, not read from the hardware. `init()` answers
false when a stated ceiling cannot be produced at this clock or a
synchronization of the peripheral did not complete. The request's
buffers are lent: the client leaves them untouched and in scope from the
post until its `SpiDone` arrives, whatever the status, and nothing
checks it. The chip select is an ordinary GPIO the device client owns:
`init()` hands DO, SCK and (when the pads name one) DI to the SERCOM and
nothing else, so the client configures its select pin as an output,
high, before its first request (the engine drives it low around each
transaction). The request names its engine, `SpiEngine::polled` the
field's default: a polled request completes inside `start()` and holds
the arbiter's dispatch, and with it every active object below the
arbiter, for the whole transfer, bounded by one spin budget; global
interrupts stay enabled. One naming `SpiEngine::dma` on a host with no
engines is refused at once - `start()` answers with `spi_rejected`, the
`SpiDone` carries it, and nothing reaches the wire; `SpiHw::has_dma` is
false there, so a program that relies on the engines states it with
`static_assert(SpiHw::has_dma)` and fails to compile. A host with
engines needs both its vectors bound ("How to use"), and spends the
image's channels from `init()` to `release()`.

The page is long because the chapter is. In order: the documents of
record; what the silicon does (the one register set, the DOPO triple,
enable protection, two characters in flight, a client's three-SCK-cycle
rule, a mode change as a CPOL flip, RXC before the wire's end, TXC,
hardware SS, the timing table against the bench, the absent event
surface, the errata); the types and verbs, the engines among them;
how to use, one example per use; the bench findings, most of the page,
where "THE TWO WAYS' COSTS" gives the length from which a request costs
the core less on the engines; and what is not covered.

## Documents of record

SAM C20/C21 data sheet DS60001479M ch. 32 (over the
shared SERCOM ch. 30) and errata DS80000740S 1.17.x. Driver:
`samc21/spi.hpp` over `Sercom<n>`'s instance facts ([sercom.md](sercom.md)
owns the address ladder, the clocks and the NVIC line - one table,
shared by every personality). Family fixture
`test/family_samc21/spi.cpp` + eleven negatives; bench suite `test_samc_spi`
(19 letters, a to s, in three images, one per arrangement of the DMA
channels: `-1` every letter but o and q, `-2` letter o, `-3` letter q, letter a
in all three - a letter is one command key of the suite's console and the
group of verdicts it runs; a to e are the registers with no wire, the
command channel to the peer, the matrix of modes and bit orders, the
rate ladder and this board as the client, and the bench findings below
name the rest - two-board: the peer board runs `spi_peer` on an AVR64EA48,
an AVR DA/DB or a second SAM C21, the suite asks ident and does not care -
commanded in band over the bus under test with the `spi_link.hpp` wire
format, one source file serving both architectures - except letters h
and i, which judge the facts of an AVR CLIENT and run only when the
ident names the avrxt peer).

## What the silicon does

**The two register views are one register set.** The device header
declares `sercom_spim_registers_t` and `sercom_spis_registers_t` with
identical offsets and field positions; the driver uses SPIM for both
roles and the role is CTRLA.MODE and nothing else. That equivalence is
static_asserted field by field at the bottom of spi.hpp, so a device
pack that ever separated them fails the build.

**DOPO is a triple, not a pad.** CTRLA.DOPO's four codes each fix a
whole (DO, SCK, SS) assignment (32.8.1); the three signals cannot be
placed independently. And which signal is which depends on the role
(table 32-2): DO is MOSI on a host and MISO on a client, so one fixed
four-wire harness is a host on one row and a client on another - two
different DOPO codes, not one code with directions flipped. The bench
runs both roles over the same four wires: a host on row 0x3 (DO PAD[0],
SCK PAD[3]) and a client on row 0x1 (DO PAD[2], SCK PAD[3]), the two
rows whose SCK pad is the XCK pad of a USART with TxD on PAD[2] (TXPO
0x1, 31.8.1) - so the same wires serve this SERCOM's USART against an
AVR's, TxD on the MISO wire and XCK on the SCK one. The header's enumerator
naming trap: `DOPO_PAD1_Val` names code 0x1, which puts DO on PAD[2] -
the same trap TXPO has.

**Enable protection and the three SYNCBUSY bits.** CTRLA, CTRLB, BAUD
and ADDR are enable-protected (32.6.2.1): a store while the instance
runs is discarded in silence - measured (a DIPO flip and a BAUD store
under a running host change nothing). SYNCBUSY has SWRST, ENABLE and
CTRLB (32.8.8); a CTRLB write while SYNCBUSY.CTRLB stands is an APB
error, so every CTRLB write in the driver waits first. Enabling the
peripheral itself raises SYNCBUSY.CTRLB (32.8.2's RXEN note): the
receiver is really up only once that second synchronization clears,
which is why `Spi<n>::enable()` waits out both.

**RXC is the byte edge, DRE is a condition.** RXC rises when a character
has been fully shifted in - on a full-duplex bus that is exactly "one
character moved, both ways" - and reading DATA is both the capture and
the acknowledgement. DRE merely means the transmit buffer is free. The
host's receive loop reads on RXC and writes on DRE; its transmit-only
loop paces on DRE alone.

**Two characters in flight is what the receive side holds.** "The SPI
is single buffered for transmitting and double buffered for receiving.
When transmitting data, the Data register can be loaded with the next
character to be transmitted during the current transmission" (32.6.1;
32.2: one-level transmit buffer, two-level receive buffer). A character
written to DATA moves to the shift register and DRE rises once it has -
three CLK_SERCOM_APB cycles after DATA empties (32.6.2.6.2's last
sentence) - and RXC rises in the clock cycle its last bit is shifted in
(32.6.2.6.1). So a host that writes k + 1 on DRE and reads k on RXC
keeps the bus busy between characters, and keeps exactly two characters
in flight - which is the receive buffer's depth: a reader held off for
longer than a character (the polled loop, by any handler of the image)
lets k complete into the buffer and then k + 1, and then the bus stops
for want of a k + 2 nobody wrote. Nothing can overflow the receiver that
way, at any rate - measured: STATUS.BUFOVF read after every receive run
of `bench_samc`'s letter e, at 12 and 3 MHz, under the console's and the
tick's interrupts, never set. The textbook loop that writes k + 1 only after
reading k leaves the bus idle for the whole turnaround: 1.26 times the
wire at 3 MHz and 2.04 at 12, where the loop a character ahead reads
1.00 and 1.14 (the bare loops, below).

**A client's DATA write needs three SCK cycles to reach the shifter**
(32.6.2.6.2), and those cycles elapse only while SCK runs. So an answer
written in the inter-character gap - where a poll loop reacting to RXC
lands - matures mid-character and reaches the wire one character late.
Measured: the host reads the preloaded first answer exactly, then the
whole stream slips by one (11 of 12 mismatched). The working client pump
is one ahead: preload b0 (CTRLB.PLOADEN puts a write made while SS is
high straight into the shifter, 32.6.3.2), park b1 in DATA at once, and
on every character boundary write the next-plus-one. So driven, the peer
read this client's 12-character answer stream byte-exact from the first
character. And the boundary is a window, not a gap: DRE rises when the
character before has moved to the shifter, three CLK_SERCOM_APB cycles
after the boundary (32.6.2.6.2's last sentence), and the next answer must
be in DATA with three SCK cycles of the current character left - so it
must land within the first five SCK cycles of every character, 17 core
cycles at 12 MHz on a 48 MHz core, 37 at 6, 77 at 3. A back-to-back
host leaves no gap to hide a late turn in: that window is what a polled
client run is written to (below).

**A mode change is a CPOL flip on the wire.** Reprogramming a host from
mode 0/1 to mode 2/3 moves SCK's idle level, and that transition is one
extra edge: landed inside an open select window, a selected client
counts it into the character - measured as an exact one-bit slip in both
directions, on modes 2 and 3 only. The engine's own `start()` applies
the request's mode before asserting the request's cs, so an
engine-owned window never sees it; a caller framing CS by hand must call
`SpiHost::prime()` first.

**RXC comes before the wire's end; TXC closes the last frame.** RXC
rises "in the same clock cycle as the last data bit is shifted in"
(32.6.2.6.1): in modes 0 and 2, whose last edge follows the sampling one
by half a bit, that is half a bit before the frame is over on the wire -
measured: a polled read that ended on RXC left SCK at its active level
at BAUD 255 (letter l's control, a loop built to end a read on its last
RXC). "In Host mode, [TXC] is
set when the data have been shifted out and there are no new data in
DATA" (32.8.6) - measured to mean the end of the last frame and not its
start: a write-only block of n frames on the transmit engine, timed from
the channel's enable to TXC in core cycles, takes n frames and a constant
at every n and rate (at 100 kHz, 2 frames 9022 cycles and 6 frames 24375:
3838 a frame against 3840; at 1 and 12 MHz the same shape). So every
request of this host ends on TXC: the polled loop's receive shapes wait
for it after their last RXC, the transmit-only phase ends on it, and so
does every request on the engines.

**Hardware SS frames a character, not a transaction.** CTRLB.MSSEN
raises SS "for a minimum of one baud cycle between each data sent"
(32.6.3.5) - measured: four rising edges on the SS pad over a
four-character transfer. A multi-byte protocol frame therefore cannot
ride hardware SS at all; the engine's chip select is an ordinary GPIO
carried in the Request (32.6.3.3's "host with several clients").

**The receive buffer is two deep.** Five characters clocked with DATA
never read: the first two survive, STATUS.BUFOVF rises (with CTRLA.IBON
it rises at the overflow; otherwise it travels with the data), and
INTFLAG.ERROR beside it. Both write-one-to-clear.

**Loop-back is real and goes through the pad.** DIPO may name the pad
DO drives (32.6.3.4): a host then reads its own transmit line back -
32 of 32 bytes identical on the bench, and nine-bit characters
(CTRLB.CHSIZE = 9) loop 0x1FF and 0x100 back whole.

**Table 45-59 agrees with the bench, and it is the stricter of the
two.** The SPI timing table (simulation values, note 1: not covered by
production test) prices the four directions at VDD > 4.5 V: host
transmission tSCK >= 2 x (tMOV 17.1 ns + the device's input setup) -
about 29 MHz against an ideal device, so the 24 MHz TX stream a
client received byte-exact is in spec; host reception tSCK >= 2 x (tMIS
50.7 ns + the device's output delay) - about 9.9 MHz against an ideal
device and 6..7 MHz against a real one, so the loop-back's measured
12 MHz exact exceeds the paper (bench-true on this die at room
temperature, not a design number) and the 24 MHz loop-back failure is
attributed: 101 ns of receiver requirement against a 41 ns period is
the receiver, by the datasheet's own arithmetic. (The 24 MHz stream
and the 12 and 24 MHz loop-back rungs were measured with a data phase on
the engines, back to back.)
A client's response
path carries tSOSS (~41 ns) plus the far end's setup plus two APB
periods (41.7 ns at 48 MHz), which lands the paper ceiling for client
transmission at a few MHz; measured against an AVR64EA48 host it holds
at 10 MHz, the fastest that host clocks, and the client's polled runs
reach it back to back (bench findings). Design guidance, then: writes to
a device up to 24 MHz, reads from a real device at 6..8 MHz by the paper
(12 MHz is what this die did), a SAM client's responses up to 10 MHz on
this bench - and a
faster SPI client is an FPGA's job, the two bus cycles in every answer
being construction, not tuning.

**There is no event surface.** 32.5.6 and 32.6.4.3 are both "Not
applicable" - this peripheral publishes nothing into `evsys.hpp`'s
fabric. There is no runtime host demotion either: the AVR's
low-SS-demotes-a-host has no counterpart here (the
role is CTRLA.MODE, written disabled); what the silicon offers instead
is CTRLB.SSDE, a client that flags/wakes on the select edge.

**Errata at rev F (E/G/J row).** 1.17.16 (SWRST inert while ENABLE = 0)
is marked on every revision and does not reproduce in SPI mode: SWRST
from the disabled state resets the block with its synchronization
completing, and even with the core clock channel really disconnected the
reset lands (bounded) once the channel returns. `Spi<n>::reset()` keeps
the enable-first discipline anyway - other SERCOM modes are unmeasured
and the cost is one enable. 1.17.19 (DBGCTRL cleared by SWRST) does not
reproduce either: DBGCTRL reads 0x1 across SWRST from both states,
exactly as 32.6.2.2 promises; configure() still writes DBGCTRL last, which is
correct under either answer. 1.17.3 (a preloaded client's first
character is a dummy unless the host holds SS low for the whole
transmission) and 1.17.20 (a preloaded character costs standby current)
are live, cannot be fixed on this side of the wire, and are stated on
`SpiConfig::preload`. 1.17.1 (spurious SSL at enable with SSDE + RXEN)
is revision B only - named in the driver precisely because it is the one
item a reader would apply without checking the row.

## Types and verbs

- **`Spi<n>`** - the resource: the whole register surface in one typed
  view (both roles), `SpiConfig` + `configure()`/`configure<cfg>()` with
  every Reserved code and every role-crossed knob refused
  (`spi_config_valid()` names whose rule each clause is), the
  synchronization waits bounded, `reset()` with the erratum discipline,
  the flags, STATUS, DATA. The baud arithmetic is pure and pinned by
  static_asserts: BAUD = f_ref/(2 f_SCK) - 1, eight bits, rounded so the
  produced rate is never above the request.
- **`SpiPads`** - the four pads and the four pins (the `UartPads`
  shape: the pad side is checked exactly - `spi_dopo_for()` answers
  with the one DOPO row or nothing - and the pin side as far as a header
  can). Role legality is checked in the role (`spi_role_probe`), because
  the same harness is legal as a host and illegal as a client on the
  same row.
- **`SpiHost<n, pads, generator, TxEngine, RxEngine, Placement>`** - the
  engine `util/spi_bus.hpp` (= `BusMaster`) drives: the shared Request
  shape (cs/dc `PinRef`s, two-phase cmd + full-duplex data,
  `Borrowed<..., Lease::reply>` spans, `ReplyTo<SpiDone>`, per-request
  BAUD value and `SpiMode`, `engine`, and `cs_setup_us` - the
  microseconds between the CS assertion and the first clock, spent
  spinning in start() in main context on samc21/delay.hpp's rate, which
  `rebase()` keeps current so it follows a clock change; measured
  spending 100 asked as 101..103). The Request is forty bytes and no
  padding on this family - the two pin references, the three spans, the
  lengths and the settings packed, the reply capsule last; the field
  names are every target's, the order this family's (the size is pinned
  by a static_assert). `apply()` compares the mode and the rate as one
  half-word against the applied one and reaches the disable/enable pair
  only on a change, so a run of requests to one device costs no register
  write at all; the SCK ceiling clamps the rate as one byte compare.
  `start()` per the bus_master contract, `baud_for()`/`sck_hz()` and the
  optional bus-wide SCK ceiling that `rebase()` re-resolves, `prime()`
  for callers that frame CS by hand.
  - **The engine slots**: `NoDmaEngine` both (the default: `has_dma`
    false), `DmaTxEngine<ch>` alone (writes on the engines, `has_dma`
    true), or both (`DmaTxEngine<a>`, `DmaRxEngine<b>`, a and b the
    budget's two channels); a receive engine alone and two engines on
    one channel do not compile (`neg/spi_receive_engine_alone.cpp`,
    `neg/spi_engines_one_channel.cpp`). Each channel is claimed for
    SERCOM n's transmit or receive request when the program is compiled,
    so a channel another request of the image owns does not compile
    ([dmac.md](dmac.md), "One owner a channel"); the engines are armed at
    `init()` and held to `release()`.
  - **`init()` starts from a reset SERCOM** (CTRLA.SWRST, the clock just
    opened, before a register of its own and before an engine is armed);
    **`release()`** lets the transmit block in flight end, puts the engines
    away, disables and gates the SERCOM, gives its pads back and closes
    its line, and resets nothing. A disable whose synchronization did not
    complete leaves ENABLE set behind the gate, the state that holds a
    request, which the next `init()`'s reset withdraws
    ([dmac.md](dmac.md), "A released requester").
  - **`SpiEngine::polled`** - the loop, inside `start()`, at any length,
    no channel touched and no vector needed. It is the chapter's host
    sequence a character ahead: the command phase and any data phase
    with an rx write k + 1 on DRE and read k on RXC (two in flight, the
    receive buffer's depth - so no rate threshold and no hazard under
    the image's interrupts). A transmit-only data phase (a null rx: a
    display's pixels) longer than the receive buffer's two characters
    paces on DRE alone, never reads the answers and drains the receive
    side at its tail; one of one or two characters takes the receive
    shape and reads its answers back, which costs less than the drain.
    Every shape ends on TXC, the receive shapes' one wait for it after
    their last RXC, and so does a command phase, which is what raises
    the D/C after the command's last edge. One budget bounds the whole
    transaction's spins, and it is a time derived from the rate when the
    rate is applied: at least twice the frames' wire time (a frame is
    16 x (BAUD + 1) core cycles) and 4096 core cycles more, counted down
    in a register at four core cycles a turn, under what a turn costs in
    the release listing. A turn runs only while the wire moves, so a
    handler over the loop spends none of it; a stalled SERCOM holds the
    dispatch for the budget's time or longer (letter k below). Nothing is flushed at start():
    every character a receive shape writes is read back by it, and the
    one shape that overflows the receiver drains it at its own tail.
  - **The one busy check** comes first on a host with engines, whichever
    engine the request names: while this host's own transfer is in
    flight on the engines `start()` answers true with `spi_rejected`,
    before the select, the D/C or the block moves, and the transfer goes
    on untouched (letter s). The arbiter never starts a request over one
    in flight, so only a program that drives the host directly meets it,
    and that program reads `status()` at once: the transfer's own end
    writes its answer there.
  - **`SpiEngine::dma`** - the engines, at any length. `start()` takes
    these steps in order. With no engines it answers `spi_rejected`;
    with nothing to move, `spi_ok`; on the transmit engine alone, a
    request that reads is `spi_rejected` - each before the registers,
    the select or the D/C are touched, `start()` answering true. A
    command followed by data runs the command in the loop and the data
    on the engines; a command alone and a data phase alone run on the
    engines. A write rides the transmit channel alone, the receiver's
    overflow drained at the end; a read starts its receive channel
    first. Both channels are silent - armed for their transfer errors
    alone - and the request's end is the SERCOM's TXC: `dma_isr()`,
    bound in the SERCOM's vector and in the DMAC's, ends it there, the
    select released after the last edge. TXC raised between two beats -
    a buffer run dry - enters the vector early, where `dma_isr()`
    answers false, and the next beat clears it. In that vector,
    TXC and DRE up with the transmit channel enabled and nothing pending
    is a channel that lost its trigger, ended `spi_dma_fault`; a bus
    error on either channel (the DMAC's vector) and a receive channel
    whose last beat does not land within 64 turns of a masked select and
    load, hundreds of core cycles against a beat's few, end the same way.
    Once the engines have the request `start()` returns false and the
    SpiDone follows from the glue in the vector; the first vector to end
    a request clears the host's in-flight mark, so the other answers
    false and one request posts one `TransferDone`. A command loop that
    stalls ends the request inside `start()` instead: true,
    `spi_stalled`, no channel armed, the D/C and the select released and
    the SERCOM reset. The request's two pins are kept for the
    end, and whether it reads; nothing else of it.
  - **At its end, ok or failed, a request leaves the D/C and the select
    high**: a clean end, a stall in either phase on either engine, a
    transfer error, and `recover()` over a request in flight (letter s). A
    refused or an empty request moves neither.
  - `status()` is the last request's word - `spi_ok`, `spi_stalled`
    (`util/spi_bus.hpp`'s code, one value on every family) when its one
    spin budget ran out with a flag never raised, `spi_dma_fault` (this
    stratum's name for `util/bus_master.hpp`'s first engine-defined
    code, `bus_engine_status`, the same value on every family with
    engines) or `spi_rejected` after a refusal.
  - **After a stall or a fault the next request starts on a clean
    block**, and the program calls nothing. A stall releases the select
    first and resets the SERCOM (CTRLA.SWRST, 32.6.2.2) inside `start()`;
    a fault, met in the vector, puts the channels away, releases the
    select and leaves a reset owed - a value of the half-word `apply()`
    compares, so the next `start()` that moves data, or `prime()`, pays
    it on the path that already asks (an empty or refused request leaves
    it owed), and neither a polled request nor a refusal carries a test
    for it. A SERCOM whose core clock stopped cannot complete
    SWRST: the debt stands, and the first request after the clock
    returns pays it. `recover()` resets the SERCOM and reconfigures it to
    the applied state, putting the channels away first: the verb a timed
    SpiBus calls on a transaction that never answered
    (util/bus_master.hpp), and a program's when it wants one. With no
    engines the host never arms the SERCOM's interrupt and leaves its NVIC
    line disabled.
- **`SpiClient<n, pads>`** - the polled surface plus ISR bodies:
  preload, SSDE, address recognition (FORM = 0x2 with AMODE/ADDR),
  `drive_output()` for a dark listener on a shared harness,
  `selected()` as a live pad read (there is no status bit for SS),
  `frames_ahead` (two: how many answers a pump must keep queued ahead
  of the host's clock, the three-SCK-cycle rule's own number - the one
  integer that differs between this family's client pump and the
  other strata's), and the polled runs - the client's unit, a
  transaction served in one call, the per-character verbs its
  degenerate case: `answer(out, spins)` loads a run of answers on DRE
  alone and reads nothing, `exchange(out, in, spins)` answers and keeps
  what the host sent, `receive(in, spins)` keeps it and writes nothing;
  each returns a `Run` (the count and how it ended: `complete`, `closed`
  - the host raised the select, TXC - or `stalled` on the spin budget).
  Their element is the character, `uint8_t` or `uint16_t` for nine bits
  (anything wider refused at compile time). They are always inline, so a
  program places the function that calls them in `.fast_text` - its call,
  as for a handler.
- **The host's placement** - util/placement.hpp's `InFlash` (the default)
  or `InRam`, the host's last parameter: it places the loops - the
  polled transaction (both phases, the D/C flip, whichever loop the
  spans pick), which start() calls once a request, and the command loop
  of a request on the engines. Under `InRam` each is one flattened
  instance in `.fast_text.brio`, a call through the linker's long-branch
  veneer from the flash ([platform.md](platform.md), "A handler in
  SRAM"); a third tag does not compile. `dma_isr()` takes the placement
  of the vector that binds it.

## How to use

A device client on the arbitrated bus, spelled the same way on every
target:

    using SpiHw = brio::SpiHost<1, my_pads>;
    using SpiBus = brio::SpiBus<SpiHw, P>;
    // in main, after the clock task's init(): if SpiHw::init(clock) answers
    // false there is no bus; then run the kernel with SpiBus in its list.
    // From an AO: take the rate from SpiHw::baud_for(6'000'000), an optional
    // checked before use, fill a Request (cs, buffers as lend<Lease::reply>(...),
    // baud, mode, engine = SpiEngine::polled), post it to SpiBus, and consume
    // the SpiDone reply.

A display's pixel stream with its loops in SRAM:

    using SpiHw = brio::SpiHost<1, my_pads, 0, brio::NoDmaEngine, brio::NoDmaEngine, brio::InRam>;

A display's pixels on the transmit engine alone (channel 1), the
console's transmitter on channel 0 beside it - the budget spent across
two transports, both vectors bound (an excerpt: `SpiArb` is the
program's `SpiBus` over this host, `Serial` its console):

    #include "samc21/dmac.hpp"
    using SpiHw = brio::SpiHost<1, my_pads, 0, brio::DmaTxEngine<1>>;
    using Serial = brio::Uart<5, console_pads, 64, 256, brio::DmaTxEngine<0>>;

    extern "C" void SERCOM1_Handler() {
        if (SpiHw::dma_isr()) { brio::post<SpiArb>(brio::TransferDone{SpiHw::status()}); }
    }
    extern "C" void DMAC_Handler() {
        (void)Serial::dma_isr();
        if (SpiHw::dma_isr()) { brio::post<SpiArb>(brio::TransferDone{SpiHw::status()}); }
    }
    // a frame of pixels: tx = the frame, rx null, engine = SpiEngine::dma;
    // a register read on this host names SpiEngine::polled (a read on the
    // engines is refused here: spi_rejected)

Reads on the engines too, the pair taking both channels - the console
beside it stays on its interrupt, the cost of which is sercom.md's (an
entry a character, about 190 cycles of the core at any rate):

    using SpiHw = brio::SpiHost<1, my_pads, 0, brio::DmaTxEngine<0>, brio::DmaRxEngine<1>>;
    using Serial = brio::Uart<5, console_pads>;

A client answering a transaction - its answers ready, the select still
high:

    Peer::init(clock, {.preload = true});
    const auto r = Peer::answer(std::span<const uint8_t>(frame), spins);
    // r.end == Peer::RunEnd::closed: the host raised the select; r.count
    // answers were loaded, and the run keeps DATA one ahead of the host,
    // so a host that closed early clocked fewer. What it sent is not kept.

The same with what the host sends kept, the run's length the shorter of
the two spans:

    const auto r = Peer::exchange(std::span<const uint8_t>(out),
                                  std::span<uint8_t>(in), spins);

## Bench findings

- The bench every finding below was measured on is the
  cross-architecture one: SAM SERCOM1 function C (PA16 MOSI, PA17 SS,
  PA18 MISO, PA19 SCK) against an AVR64EA48 peer's SPI0 ALT1 (PE0 MOSI,
  PE3 SS, PE1 MISO, PE2 SCK) plus GND, both boards at 5 V. The suite
  also serves a SAM-SAM bench (both boards' PA16..PA19 straight through
  plus GND), not yet run ("Not covered yet"). On either the same wires
  carry a board as host (DOPO row 0x3) and as client (row 0x1).
- All four transfer modes x both bit orders byte-exact in both
  directions; a deliberate DORD mismatch is an EXACT two-way bit
  reversal at both ends.
- THE SCK LADDER ON THE WIRE (letter f, the loop-back through PA16, no
  wire; [../design/overview.md](../design/overview.md), "A loop proves
  the bytes, never the rate"): a frame inside a polled block - the
  difference of a 1024- and a 128-frame block in the transmit-only shape,
  the best of three, so a request's fixed cost cancels - against its
  eight SCK periods at the rate ASKED, on the 24 MHz crystal (TC2+TC3,
  41.7 ns). Every rung from 93.75 kHz to 6 MHz reads 1006 thousandths and
  8 MHz 1007, mode 3 at 1 MHz 1006, and letter g's two rates the same.
  The reading does not move with the rate, so there is no gap between
  frames (DATA loaded while the frame before it shifts, 32.6.1) and the
  six thousandths are OSC48M against the crystal - the oscillator the
  SERCOM runs on, half a per cent off on this board
  ([../boards/samc21j.md](../boards/samc21j.md)); the divisor is exact.
  12 MHz reads 1038 to 1041 and 24 MHz 1695 in three runs: there the
  frame (32 and 16 core cycles) is no longer than the polled loop's
  turn in this image - 33 and 27 core cycles - so the reading is the
  loop's and not the wire's (`bench_samc`'s image turns the same loop
  in 32.0 at 12 MHz, below; a cycle a frame between two images, not
  attributed).
- Back-to-back characters (no inter-byte gap - one engine request)
  bind at the PEER: its answer reload, or its silicon under it. The AVR
  peer serves each window with one call of its driver's runs
  ([../avrxt/spi.md](../avrxt/spi.md), the client's runs): exact both
  ways to 2 MHz in every ladder of letter d, and at 3 MHz every answer
  that is wrong is wrong in its first bit alone while the client
  receives every character exact - the AVR's clarification 3.5.2, its
  buffer's copy into the shifter later than this host's first edge. No
  window stopped short: letter i's 36 windows of 256 characters inside
  the client's ceiling, in each of ten runs against the AVR64EA48.
- THIS CLIENT BACK TO BACK (`test_avr_spi` letter u: an AVR64EA48 host,
  sixteen characters in one polled request of its engine - its own pace
  2 CLK_PER between characters at CLK_PER/4 and about 7 at CLK_PER/2 -
  at 1.25, 2.5, 5 and 10 MHz, its 20 MHz crystal's divisions; at each, a
  GAPPED control - one character at a time, 20 us between them - then the
  exchange run, then the answer run). With the peer's windows on
  `exchange()` and `answer()` placed in SRAM, both runs are exact back to
  back at every rung to 5 MHz in each of eleven ladders, and at 10 MHz in
  ten of eleven (one full-duplex run with 7 answers wrong, the gapped
  control and the answer run exact): the exchange run at the host's 10
  MHz is at its edge. THE COST, counted in the release listing: the
  answer run polls INTFLAG four times a count of its budget, each poll a
  load, a shift into the sign bit and a branch; from the read that sees
  DRE to the store in DATA is that load, a taken branch and the store -
  7 instruction cycles beside the two peripheral accesses, 11 at worst
  with one poll more, against the window's 21 core cycles at 10 MHz
  (the five SCK cycles above); nothing in the loop is a call. The exchange run adds the RXC
  poll and the DATA read and store: 23 instruction cycles a turn beside
  four peripheral accesses, against a character of 38 at 10 MHz - the
  margin the one broken run above ate. So the peer places its runs in
  `.fast_text`: from the flash every fetch of the turn waits on
  NVMCTRL's read wait states.
- The kernel letter (g): four polled requests queued in one dispatch
  come back through their own ReplyTo with `spi_ok`, every byte of the
  loop-back exact; an asynchronous request posted between two polled
  ones comes back `spi_rejected`, its receive buffer untouched and the
  arbiter's own rejection count still zero, and the bus carries the next
  one; an idle bus votes ok on PrepareSleep, and so does a vote posted
  right behind a request, the request's reply ahead of it - each request
  ends inside its own dispatch, so the arbiter never holds one in flight
  over this host and its pending FIFO never fills (the reject-when-full
  and the busy vote are the arbiter's, host-tested against an
  asynchronous engine in `test_bus_master`). Ten runs of g, every one
  so. `util/spi_bus.hpp` and `util/bus_master.hpp` arbitrate this engine
  as written: nothing above the contract is target-specific.
- Letter h, the AVR CLIENT'S FACTS with this board as the host: every
  verdict `test_avr_spi` draws against an AVR client holds with a SAM
  C21 clocking it - the three regimes' alignment (the dummy 0x00), the
  CLK_PER/6 ceiling - 3.33 MHz on the AVR64EA48's 20 MHz - (exact at 3
  and 1.5 MHz; at 6 MHz the client received 12 of 12 wrong and 9 to 12 of
  its answers arrived wrong, at 12 MHz every character wrong both ways,
  the client counting none), what it
  keeps undrained (normal the last of eight, buffer the first two and
  the last, BUFOVF only with its transmitter fed), the write-collision
  boundary and the missed load, and its self-selected client in the
  four modes and LSb first.
- Letter i, the AVR client's RUNS back to back: 256 characters as one
  polled request of this engine at 500 kHz, 1, 2, 3 and 4 MHz - six
  exchange windows a rung, an answer-only and a receive-only one. Exact
  both ways through 2 MHz; at 3 MHz the client receives every character
  of every window and its answers - the exchange's and the answer run's,
  which reads nothing - arrive wrong in the first bit alone, the MSB of
  the character just received. 4 MHz is above the AVR64EA48's CLK_PER/6
  (3.33 MHz), and there the client mis-samples MOSI: 143 to 154 of 256
  received wrong, ten runs of ten. Without BUFWR
  (regime_buffer) the exchange run is exact both ways through 2 MHz too,
  this host reading the leading dummy and then every answer one place
  late, as spi_link.hpp's table says.
- THIS CLIENT'S RUNS AGAINST A HOST'S BURST (letter j: the AVR64EA48
  peer's host_burst, one character at a time with 20 us between them, at
  its CLK_PER/32, 625 kHz; a character and its gap about 39 us, measured
  from the select's fall to the run's end): the exchange run complete and exact both ways;
  the receive run complete and exact, the host reading the echo one
  character late (32.6.2.6.2); `stalled` both ways, with nothing - a run
  nobody selects (40 core cycles a count of its budget) and one whose
  select the peer holds low with no clock (its ss_pulse), the select
  still low at the end. INTFLAG.SSL rises once at the select's fall with
  CTRLB.SSDE (32.6.3.6). Address recognition (FORM 0x2, 32.6.3.1): a
  burst opening with the address is received whole, the address the
  run's first character; one opening with another is IGNORED WHOLE -
  nothing received, TXC never raised (32.8.6), the run ending on its
  budget.
- A HANDLER TIMED INTO A RUN (letter j: the SSL interrupt itself holds
  the core). One that ends before the first character changes nothing.
  One that outlasts an answer's window - due within the first five SCK
  cycles of character 1, 62 us after the fall, the handler 75 us, ending
  before character 2 starts at 90 us - LOSES NO CHARACTER: character 2
  carries what the shifter holds, character 1 as received, and every
  later answer goes one character late - ten of twelve wrong - while the
  client receives every character exact and the run ends `complete`. No
  flag marks a reload that matured a character late, so the run cannot
  see it; masking around a run is the program's call.
- THE SELECT'S FALL WAKES THE CORE (letter j): from IDLE0, and then from
  STANDBY with CTRLA.RUNSTDBY set (32.6.5), one WFI each with the tick's
  interrupt off, SERCOM1's the line pending at the wake and the RTC's
  backstop silent; a receive run started on the wake takes the burst
  whole, the 20 us gap ahead of the first character. The console's own
  transmitter wakes a standby too: a line still draining woke it once a
  character (SERCOM5's DRE), so the letter drains it first.
- THE HOST'S RARE PATHS (letter k). The spin budget: a polled request of
  eight with the core clock channel disconnected at BAUD 119 comes back
  spi_stalled after 44 ms: the budget, 8704 turns of the flag wait or about
  3 ms at some 16 core cycles a turn (its floor is 34,816 core cycles, twice
  the frames' wire time and 4096 more), and then the reset inside `start()`,
  whose wait for a synchronization a stopped clock never completes is a
  fixed 65535 turns (`Spi::reset()`, a setup verb's bound and not the wire's),
  the select raised and the receiver empty, and the engine runs the next
  request once the channel is back. An early TXC: 64 characters at 1 MHz
  in the transmit-only shape under a 60 us handler about 120 us in -
  TXC stood at the handler's end, the shifter run dry under it, the next
  write cleared it (32.8.6) and the tail waited for the last character's:
  the peer's receive run counted 64 exact and ended complete, the select
  held to the last. Ten runs of k, every one so.
- A REQUEST'S END (letter s, no wire and no peer). The D/C on a real pad
  (PB23), read after the end, the select on a spy group in SRAM: a command
  stalled on a clockless SERCOM, polled with data behind it, polled alone
  and in the loop ahead of the engines, and a data phase stalled, each end
  `spi_stalled` with the D/C high and the select fallen and risen; a command
  alone on the engines whose source runs past the SRAM's end ends
  `spi_dma_fault` the same; `recover()` over a command in flight, the D/C
  read low in it, leaves both high. THE ONE BUSY CHECK: 64 frames on the
  engines at BAUD 255, a polled request and a request on the engines each
  started over them answer true with `spi_rejected`, no store on either spy
  pin and CTRLA unchanged, and the transfer ends `spi_ok`, every byte exact on
  the loop-back. THE D/C'S RISE, a command of two at BAUD 255 (half a bit is
  256 core cycles), sixteen requests a row, a timer's vector in SRAM sampling
  the D/C and then the SCK pad every 160 core cycles: no sample with the D/C
  high and SCK active, a command alone in the four modes, and the first
  sample with the D/C high finding SCK idle with data behind the command in
  modes 0 and 2. With the wait on TXC taken out of the host the same rows
  fail 15 or 16 requests of 16 in modes 0 and 2 and none in modes 1 and 3,
  which is RXC's half bit seen from the D/C. The sampler is too slow for the
  loop ahead of the engines, whose way back to the D/C's store outlasts half a
  bit under it, so that row is printed and not judged, and that path runs the
  one `command_phase()` the polled rows judge. Ten runs of s, every one so.
- THIS SILICON AS THE CLIENT, measured against an AVR64EA48 host
  (`test_avr_spi` y with the samc21 `spi_peer`, one character at a time
  with 20 us between them, the select held for the window):
  - THE REGIMES: with PLOADEN (the peer's regime_buffer_wait) the answer
    stream arrives from the first character; without it (its normal and
    buffer) a DUMMY leads, 0x00 after a fresh init, and the stream is one
    place late - 8 of 8 combinations each, every mode and bit order.
  - THE RATE: exact both ways from CLK_PER/128 to CLK_PER/2 of the
    host's 20 MHz - 156.25 kHz to 10 MHz, the fastest that host clocks -
    in ten runs of ten (the paper's "a few MHz" above is the conservative
    side of it, the host's own input sampling part of the answer's path).
  - UNDRAINED (eight characters at 750 kHz with DATA never read): the
    two-level receive buffer keeps the FIRST TWO and the rest is lost; the
    drain then reads one more 0x00 and INTFLAG.ERROR stands after it -
    the overflow travelling with the data (CTRLA.IBON = 0, 32.6.2.7);
    a fed transmitter changes nothing. The AVR's write collision and
    missed-load echo have no counterpart: a DATA write here goes to the
    buffer, the stream is untouched.
  - The select raised MID-CHARACTER: the client keeps the three complete
    characters and drops the interrupted one (the client's shift counter
    resets at SS high, 32.6.2.6.2).
  - A CPOL mismatch corrupts what the client receives (12 of 12), a CPHA
    mismatch what the host reads (11 of 12); a DORD mismatch is the exact
    two-way reversal.
- THE SELECT PAD'S PULL IS A PULL-DOWN. Under PMUXEN, PINCFG.PULLEN on
  the client's SS pad gives a pull-DOWN whatever OUT says - the rule
  31.5.1 states for the USART, measured here on the SPI: with the far end
  a high-impedance input, the line read low within a millisecond of
  PULLEN with OUT set, and stayed high for 300 ms with no pull. A pull
  there SELECTS the client, and against an AVR host watching its SS
  (SSD = 0, its own pull-up on) it made a divider that read low and
  demoted the host. So a client here keeps no pull on its select, and
  the host's end holds the line.
- THE ANSWER WINDOW ENDS ON TXC. In a client INTFLAG.TXC rises when SS is
  pulled high (32.8.6) and a DATA write clears it, so a polled answer
  window ends when the host raises the select, whatever count it
  clocked - a host that stops at its decoder's frame (the AVR's) would
  otherwise have its next window eaten as dummies of this one.

- THE SELECT'S RULE (letter l, the loop-back, a stand-in select on PB23
  and none): when a request ends the wire has ended - right after the
  end, SCK reads at its idle level - in the four modes, read and write,
  polled and on the engines, at BAUD 255 (93.75 kHz, half a bit 256 core
  cycles) and BAUD 23, ten runs of ten. The bound: an end that came with
  the last RXC would read SCK at its active level in modes 0 and 2 for
  the last half bit, and the pad is read within a few tens of cycles of
  the end, so at BAUD 255 it cannot be missed; at BAUD 23 the half bit is
  24 cycles and the instrument no longer sees it. The control, the same
  letter built with a loop that ends a read on its last RXC: four requests of
  thirty-two ended before the last edge at BAUD 255 - the mode 0 and mode
  2 reads - in each of four runs. The cost of the rule: one TXC wait a
  polled request, a flag already up on the shapes that stopped on it, and
  at most half a bit on a read; on the engines nothing, TXC being the one
  interrupt anyway.
- THE ENGINES (letter m, against the AVR64EA48's `spi_peer` at 1 MHz -
  a frame 384 core cycles, the loop far cheaper there - but for its
  last two cases, on the loop-back at 6 MHz): one frame written
  on the transmit engine alone and one frame read on the pair, each
  `start()` false and `dma_isr()` the end, `spi_ok`, exact at both ends;
  sixteen read in mode 3; a read naming `dma` on the transmit engine
  alone, `dma` on a host with no engines and an empty request naming
  `dma` each answering at once with nothing stored to the select or the
  D/C (a select and a D/C on a spy group in SRAM, so the host's OUTCLR and
  OUTSET land where the letter reads them) and no frame on the wire, the
  next request exact; a command of one then sixteen data, the command in
  the loop and the data on the engines, the peer receiving all seventeen
  in order, the D/C cleared and then set; a command of four alone on the
  engines, received whole, the D/C high at its end as a polled one leaves
  it; then on the loop-back at 6 MHz (BAUD 3, a frame 64 core cycles,
  32 ticks of the crystal ruler), 1024 polled frames on an engined host
  never writing a channel's slot, and a command of one with 65535 data
  frames - 65536, which a sum taken in sixteen bits would read as an
  empty request - taking the wire's 2,097,152 ruler ticks (65536 frames
  of 32) and more, polled (2,119,964) and on the engines (2,111,231).
  Ten runs of m, every one so.
- THE BLOCK'S RESET (letter n, the loop-back): a polled request on a
  clockless SERCOM ends `spi_stalled` and the next read of sixteen, the
  clock back and nothing called, is exact; a read on the pair whose
  transmit run crosses the SRAM's end (0x20008000) ends `spi_dma_fault`
  and the next read is exact on the engines and polled. Ten runs of n,
  every one so. The control, the same letter with the reset taken out:
  sixteen of sixteen wrong after the stall - the character the stall left
  in DATA shifted out once the clock returned, its echo leading the next
  read - and sixteen of sixteen after each fault.
- THE TWO SHAPES A BOARD FILE CAN CHOOSE, held ten minutes each with
  every class of fault counted (letters o and p): THE DISPLAY'S - the
  transmit engine alone on channel 1 beside the console's transmit engine
  on channel 0, both running at once: 1357 windows of 256 frames written
  at 2 MHz into the AVR64EA48's receive run, exact at the peer, and 5428
  blocks of 1024 at 12 MHz on the loop-back, every request ended `spi_ok`
  on the engines, no block abandoned, the console's 600 marked lines read
  whole by the host; THE PAIR'S - both channels the SPI's, the console on
  its interrupt: 15,790 windows of sixteen frames read and written at
  2 MHz with the peer and 63,160 reads of 1024 at 12 MHz on the loop-back,
  every byte exact both ways, every request `spi_ok`, no block abandoned,
  600 console lines whole. The console beside a pair costs its interrupt
  transmitter, some 190 cycles an entry a character (sercom.md).
- THE TWO WAYS' COSTS (`bench_samc` letter e, the pair at 3 MHz, 64
  frames, best of 8, three runs): E, the engines' least busy cycles of
  the core - `start()`, the one TXC vector, and the turns of the loop
  the bench waits in (interrupts masked while it tests the end, the
  core asleep between two tests) - 999 written and 979 read from the
  flash (897 and 865 with the vectors and the loops in SRAM); F, the
  polled request's rest above the wire, 457 to 458 and 374 (423 and 346
  to 351); E - F, 541 to 542 written and 605 read (474 and 514 to 519).
  The engines' wall carries the vector's latency, x 1.13 of the wire at
  64 frames and x 1.03 at 1024 frames at 12 MHz. A polled request holds
  the core for its frames' wire time - or the loop's turn where that is
  longer: about 27 cycles a frame written at 24 MHz (letter f), 41 read
  at 12 MHz - so the engines cost the core fewer cycles from the length
  (E - F) / frame:

  | SCK (BAUD) | frame, core cycles | engines cheaper, written | read |
  |---|---:|---:|---:|
  | 24 MHz (0) | 16 (the loop's 27) | 21 frames | - (beyond the receiver) |
  | 12 MHz (1) | 32 (read: the loop's 41) | 17 | 15 |
  | 6 MHz (3) | 64 | 9 | 10 |
  | 3 MHz (7) | 128 | 5 | 5 |
  | 1.5 MHz (15) | 256 | 3 | 3 |
  | 750 kHz (31) | 512 | 2 | 2 |
  | 375 kHz and slower | 1024 and up | 1 | 1 |

  Guidance, which nothing in the code reads: at 3 MHz and slower a
  request of five frames or more costs the core less on the engines, at
  12 MHz from about sixteen and at 24 MHz from about twenty written;
  below that the loop is cheaper, and a wrong choice costs time, never
  correctness.
- THE WRITE AT 24 MHz (letter r, the transmit engine alone on the
  loop-back, blocks of 64 and 1024 frames, the frame from their
  difference so the request's fixed cost cancels, on the crystal ruler -
  OSC48M, which the SERCOM runs on, reads six thousandths fast against
  it): a frame takes 32.20, 64.40 and 128.8 core cycles at BAUD 1, 3 and
  7 - the wire's 32, 64 and 128 - and at BAUD 0, against the wire's 16,
  19.11 to 19.12 with the source in the SRAM, the core spinning on a
  flag or asleep, and with the source in the flash 19.29 to 19.31
  asleep and 19.59 to 19.72 spinning (two runs): 20 Mbit/s on a 24 MHz
  SCK. So the gap is mostly neither the source's memory nor the core's
  use of the bus (the two together at most a fifth of it, and only with
  the source in the flash) but the channel's own
  turn: DRE rises three CLK_SERCOM_APB cycles after DATA empties
  (32.6.2.6.2) and the next beat lands about sixteen cycles after it -
  the trigger's way into the controller, the arbitration, a read and a
  write through the APB bridge - where a 16-cycle frame leaves thirteen.
  At BAUD 1 and slower a frame has room for it and the engine keeps the
  wire's pace. (The split of the sixteen between those stages is
  inferred, not measured: a scope on SCK would show the idle cycles
  between two frames.)
- `bench_samc` letter e, THE HOST ABOVE THE WIRE: the host on SERCOM1
  with MISO floating (the time is the wire's), the best of 8 in each of
  three runs, the instrument's figures in the same image (its letter r)
  being ruler 65, stamp 148 (56 of it charged to a handler), interval
  82. The polled loop (`spi.poll` the write shape, `spi.poll.rx` the
  receive one; a character's cost is the difference of the 256- and the
  16-frame lines over 240, so the request's fixed cost cancels): at
  3 MHz the wire's, x = 1.01 for 256 frames in both shapes; at 12 MHz
  the write shape 32.0 cycles a character, the wire's 32 (x = 1.05 for
  256 frames, the fixed cost inside), the receive shape 43.0 (x = 1.38; the
  same instructions read 41.0, x = 1.32, at another place in the flash of
  another image, and 37.0 in SRAM in both: the difference is the fetch's).
  The data sheet's own sequence written bare over the registers in a
  scratch program on the same board - no library exists for this
  family - is the oracle: a character ahead, reading every answer back,
  it reads x = 1.14 at 12 MHz (36.6 cycles a character, 14 instructions
  a turn) and 1.00 at 3; the textbook loop with one in flight 2.04 and
  1.26. THE CORTEX-M0+ AT 48 MHz BEHIND TWO WAIT STATES OF FLASH CANNOT
  TURN A TWO-FLAG POLL AROUND INSIDE A 32-CYCLE CHARACTER, whatever the
  driver: the write shape keeps the wire because it polls ONE flag; what
  separates brio's receive loop from the bare one is the flash (the
  same loops under `InRam`: 32.0 and 37.0 cycles a character, x = 1.05
  and 1.20 for 256 frames), not the budget - a register count-down
  against a stack slot moved nothing.
  STATUS.BUFOVF after every run: never. There is no interrupt engine:
  the other way is DMA ([design/spi-bus.md](../design/spi-bus.md), "Two
  ways to run a request", weighs a per-character interrupt against the
  character); what a polled window costs the latency of a higher object
  is [platform.md](platform.md)'s ("The drivers under the preemptive
  kernel"). THE FIXED COST OF A
  REQUEST (`spi.req`: a polled request of 1, 3 and 16 bytes with a
  command byte, the D/C scripted on PB23 and the select on PA17 - two
  real pads, four real edges - at 3 MHz): 545, 576 and 664 cycles above
  the wire (wall 673, 960 and 2712), the instrument's 82 inside, the same
  in each of three runs. The wait for TXC that raises the D/C after the
  command's last edge shows on a request of one and on none of the longer
  two: with that wait and the loop's other changes out of the host the walls
  read 665, 961 and 2714. At this rate half a bit is 8 core cycles, and at
  BAUD 255 it is 256; a request with no command never meets it.
  The phase boundary is the 31 between one byte and three (the last
  command character read back and closed on TXC, the D/C, the first data
  character's own start). The 88 from three to sixteen is the transmit-only phase's
  tail - TXC awaited and the receiver drained - which a data phase
  longer than the receive buffer pays; a data phase of two characters
  reads its answers back instead and pays no drain. What it is made of,
  read in the listing and measured with the polled transaction `InRam`
  (20, 21 and 37 cycles less, the call through the veneer included, so
  the flash is not the bulk of it): the bench's own request built on the
  stack and the interval; start() along that path - the refusal's test
  (the field's load, a compare and a branch, then the `spi_ok` store),
  the length test, the one-word compare, the four pin edges inline
  through `PinRef::set/clear` (pin.hpp forces the two verbs inline: each
  edge is the group pointer's load, the null test, the mask's load and
  one OUTSET or OUTCLR store - as calls they cost 49 cycles more on the
  3-byte request, measured), the call to the transaction and the
  return; the loop's prologue; the wait for TXC at the request's end
  that the select's rule asks (`test_samc_spi` letter l, above) and its
  test; and the
  SERCOM's own start and finish latencies at the boundaries. Three runs
  of each image, the best of 8 in each.

## Not covered yet

Driver gaps (not built): **a DMA data phase on the client** - the
client's runs are polled; born with a client whose host clocks faster
than its polled run turns. **A read on the engines with the transmit
engine alone** - a host that reads on the engines names both, and a
receive engine alone does not compile: the transmit channel's beats are
what clock the wire. **Nine-bit frames on the host** - the host's
Request is bytes and its frame eight bits; born with a nine-bit device.

Driver gaps, continued: **the pin edges through the IOBUS alias** -
`PinRef::set/clear` store through the APB bridge, inline (four stores a
request); the single-cycle IOBUS alias (10.1.4) is pin.hpp's and would
take some 12 cycles more off a request's fixed cost, not the hundreds
between it and 300: declined here because the rest of that cost is the
request's shape and the SERCOM's own latencies, stated above.

Implemented but not bench-verified:

- The D/C's rise after a command in the loop ahead of the engines: it runs
  the same `command_phase()` as the polled rows of letter s, whose sampler
  reacts too late there to fail without the wait; a logic analyzer on the
  D/C and SCK would measure it.
- The SAM-SAM pairing on the harness's rows (0x3 host, 0x1 client), and
  the samc21 `spi_peer`'s window end and unpulled select against a host
  that clocks the whole answer budget (`test_samc_spi` itself): needs a
  second SAM C21 at 5 V; the AVR64EA48 pairing on these rows is
  measured, both ways.

- The split of the 24 MHz write's three extra cycles a frame between the
  trigger's way into the controller, the arbitration and the beat (Bench
  findings): a scope on SCK would show the idle cycles between frames.
- Erratum 1.17.20's standby cost: a client that preloaded before a
  standby may draw extra current - a meter on the supply would measure
  it, and the desk has none on this board.
- On silicon: 9-bit characters through a real two-board transfer
  (loop-back only), and every client run on them - the peer that clocks
  this board is an AVR64EA48, eight bits only, so a second SAM C21 would
  measure them; IBON = 0's travelling overflow flavour.
- A client run's rarest end: the select raised in the instructions
  between the poll's read and the store it decides, which the store's own
  clear of TXC turns into `stalled` (the method's statement; staging it
  wants a host that raises its select within a few core cycles of a
  character boundary).
- Erratum 1.17.3's dummy-first-character: it needs a client still
  preloaded across a select the host raised MID-TRANSMISSION and dropped
  again, and spi_link.hpp has no such host op (host_burst holds the
  select for the whole burst), while a peer's own runs end at the first
  close; an op that raises the select inside a character and selects
  again would measure it.

Stated, not enforced: `SercomPadPin`'s pin-reaches-pad claim is the
caller's (sercom.md's open device-table question, unchanged here).
