# SERCOM SPI (SAM C21)

This document is for a programmer who needs an SPI bus on the SAM C21:
a host talking to one or more devices (a sensor, a flash chip, a
display) through the bus arbiter, or this chip answering as a client
to another board's host. `samc21/spi.hpp` is the SERCOM's SPI mode in
three parts. `Spi<n>` is the resource, one function per register fact
of one SERCOM instance. `SpiHost<n, pads, ...>` is the transfer engine
`util/spi_bus.hpp` drives, a task (in brio, a driver layer with state
of its own, served from its peripheral's interrupt) whose Request
carries the same field names on every target, so a device client reads
unchanged on any of them. `SpiClient<n, pads>` is the client role. The
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
// in main: SpiHw::init(clock); kernel.init_all(); ...
// ISR glue:
extern "C" void SERCOM1_Handler() {
    if (SpiHw::isr()) { brio::post<SpiBus>(brio::TransferDone{brio::spi_ok}); }
}
// from an AO: fill a Request (cs, buffers as lend<Lease::reply>(...),
// baud = *SpiHw::baud_for(6'000'000), mode), post it to SpiBus, and
// consume the SpiDone reply.
```

Line by line: `SpiHw` names the engine over SERCOM1 with the pads
`my_pads` (an `SpiPads` constant naming the four SERCOM pads and the
pins they are bonded to, "Types and verbs" below), every other template
argument at its default - generator 0, the SERCOM's core clock at the
CPU rate, and `InFlash`, no path placed in SRAM; `SpiBus` is the arbiter
over that engine on the platform `P` ([platform.md](platform.md)), with
`util/spi_bus.hpp`'s defaults of four pending requests and no timeout;
`SpiHw::init(clock)` takes the clock task's tag ([clock.md](clock.md))
and brings the instance up as a host, and the rest of `main()` starts
the kernel with `SpiBus` in its list of active objects (`run()` calls
every active object's `init()`, which is what `init_all()` names); the
`extern "C"` function is the SERCOM1 interrupt handler, as in C, under
the name the crt's vector table gives it, and `SpiHw::isr()` answers
true when an interrupt-driven transaction has just completed, the edge
on which the handler posts `TransferDone` to the arbiter; and an active
object (AO) fills a `Request` - the chip select as a `PinRef`, the
buffers lent with `lend<Lease::reply>(...)`, the BAUD register value
`baud_for()` computes for the device's 6 MHz, the mode - posts it to
`SpiBus` and receives `SpiDone` with the status (`spi_ok`, or
`spi_stalled` when a polled request's spin budget ran out).

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
transaction). A request with `polled = false` (the default) runs on the
interrupt, so the SERCOM's vector must be bound to `isr()` as above; an
unbound vector is a weak alias of the crt's silent spin, where the
program stops at the first character. A polled request completes inside
`start()`, silences the SERCOM's own interrupt and holds the arbiter's
dispatch, and with it every other active object, for the whole transfer,
bounded by one spin budget; global interrupts stay enabled.

The page is long because the chapter is. In order: the documents of
record; what the silicon does (the one register set, the DOPO triple,
enable protection, two characters in flight, a client's three-SCK-cycle
rule, a mode change as a CPOL flip, TXC, hardware SS, the timing table
against the bench, the absent event surface, the errata); the types and
verbs; how to use, one example per use; the bench findings, most of the
page; and what is not covered.

## Documents of record

SAM C20/C21 data sheet DS60001479M ch. 32 (over the
shared SERCOM ch. 30) and errata DS80000740S 1.17.x. Driver:
`samc21/spi.hpp` over `Sercom<n>`'s instance facts ([sercom.md](sercom.md)
owns the address ladder, the clocks and the NVIC line - one table,
shared by every personality). Family fixture
`test/family_samc21/spi.cpp` + nine negatives; bench suite `test_samc_spi`
(9 letters - a letter is one command key of the suite's console and the
group of verdicts it runs - two-board: the peer board runs `spi_peer` on an AVR64EA48,
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
host engine arms RXC and never DRE: one interrupt per character.

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
longer than a character (the polled loop by any handler of the image,
the pump's handler by a late entry) lets k complete into the buffer and
then k + 1, and then the bus stops for want of a k + 2 nobody wrote.
Nothing can overflow the receiver that way, at any rate - measured:
STATUS.BUFOVF read after every polled and every pumped run of the
bench's letter e, at 12 and 3 MHz, under the console's and the tick's
interrupts, never set. The textbook loop that writes k + 1 only after
reading k leaves the bus idle for the whole turnaround: a fifth slower
at 3 MHz and twice as slow at 12 (the bare loops, below).

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

**TXC closes the last frame.** "In Host mode, this flag is set when the
data have been shifted out and there are no new data in DATA" (32.8.6) -
measured to mean the end of the last frame and not its start: a
write-only block of n frames fed back to back by a DMA channel (a
measurement of the silicon, not a path of this driver), timed from the
channel's enable to TXC in core cycles, takes n frames and a constant at
every n and rate (at 100 kHz, 2 frames 9022 cycles and 6 frames 24375:
3838 a frame against 3840; at 1 and 12 MHz the same shape). So a
write-only transaction needs no reading side to know it is over: TXC is
its edge, and the polled transmit-only phase ends on it.

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
and the 12 and 24 MHz loop-back rungs were measured with a DMA-fed data
phase, back to back: facts of the silicon, not paths of this driver.)
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
- **`SpiHost<n, pads, generator, Placement>`** - the engine
  `util/spi_bus.hpp` (= `BusMaster`) drives: the shared Request shape
  (cs/dc `PinRef`s, two-phase cmd + full-duplex data, `Borrowed<...,
  Lease::reply>` spans, `ReplyTo<SpiDone>`, per-request BAUD value and
  `SpiMode`, `polled` completion style, and `cs_setup_us` - the
  microseconds between the CS assertion and the first clock, spent
  spinning in start() in main context on samc21/delay.hpp's rate, which
  `rebase()` keeps current so it follows a clock change; measured
  spending 100 asked as 101..103). The Request is forty bytes and no
  padding on this family - the two pin references, the three spans, the
  lengths and the settings packed, the reply capsule last; the field
  names are every target's, the order this family's - and it is lent
  for the call (util/bus_master.hpp): a polled request is read through
  the reference and copied nowhere, an asynchronous one is copied once
  into the engine right before its first character goes out, forty bytes
  of inline load-multiple/store-multiple (the compiler expands a struct
  assignment of that size inline where it called memcpy for forty-eight,
  and calls memcpy for a built-in copy of thirty-two: the size is pinned
  by a static_assert for that reason). `apply()` compares the mode and
  the rate as one half-word against the applied one and reaches the
  disable/enable pair only on a change, so a run of requests to one
  device costs no register write at all; the SCK ceiling clamps the
  rate as one byte compare. `start()`/`isr()` per the bus_master
  contract, `baud_for()`/`sck_hz()` and the optional bus-wide SCK ceiling
  that `rebase()` re-resolves, `prime()` for callers that frame CS by
  hand. The polled loop is the chapter's host sequence a character
  ahead: the command phase and any data phase with an rx write k + 1 on
  DRE and read k on RXC (two in flight, the receive buffer's depth - the
  silicon fact above, so no rate threshold and no hazard under the
  image's interrupts); a transmit-only data phase (a null rx: a
  display's pixels) paces on DRE alone, never reads the answers, ends on
  TXC - which closes the last character (below) - and drains the receive
  side at its tail. One budget bounds the whole transaction's spins,
  counted down in a register. The pump arms RXC and writes k + 2 from the
  handler that reads k, so two are in flight there too; the first data
  character waits for the last command one to be back (the D/C must
  change on the wire between them), one in flight across that boundary
  alone; the first two characters go out before the interrupt is armed
  (the handler and start() advance the same write index, and at 12 MHz
  the first character is back before start() has written the second).
  Nothing is flushed at start(): every character the pump or a receive
  shape writes is read back by it, and the one shape that overflows the
  receiver - the polled transmit-only phase - drains it at its own tail.
  There is no DMA data phase: the polled loop and the pump are the two
  shapes (dmac.md, erratum 1.10.4). `status()` is the completion's word
  - `spi_ok`, or `spi_stalled` (`util/spi_bus.hpp`'s code, one value on
  every family) when a polled transaction's one spin budget ran out with
  a flag never raised. `recover()` is the verb a timed SpiBus calls on a
  transaction that never answered (util/bus_master.hpp): CS deasserted
  first, the SERCOM reset and reconfigured to the applied state - the
  wedge it targets is an ISR-style completion that never posts (a lost
  interrupt).
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
  or `InRam`, the host's last parameter: two paths, each one function of
  it - the polled transaction (both phases, the D/C flip, whichever loop
  the spans pick), which start() calls once a request, and the pump's
  entry, which the vector calls. Under `InRam` each is one flattened
  instance in `.fast_text.brio`, a call through the linker's long-branch
  veneer from the flash ([platform.md](platform.md), "A handler in
  SRAM"); a third tag does not compile. The polled transaction is the
  driver's alone to place; the pump's entry the program can place better
  by placing its vector, and both together cost the pump a call (the
  cost below).

## How to use

A device client on the arbitrated bus, spelled the same way on every
target:

    using SpiHw = brio::SpiHost<1, my_pads>;
    using SpiBus = brio::SpiBus<SpiHw, P>;
    // in main: SpiHw::init(clock); kernel.init_all(); ...
    // ISR glue:
    extern "C" void SERCOM1_Handler() {
        if (SpiHw::isr()) { brio::post<SpiBus>(brio::TransferDone{brio::spi_ok}); }
    }
    // from an AO: fill a Request (cs, buffers as lend<Lease::reply>(...),
    // baud = *SpiHw::baud_for(6'000'000), mode), post it to SpiBus, and
    // consume the SpiDone reply.

A display's polled pixel stream with its loops in SRAM, and the pump's
vector placed by the program (the entry inline in it, with no call):

    using SpiHw = brio::SpiHost<1, my_pads, 0, brio::InRam>;
    extern "C" [[gnu::section(".fast_text")]] void SERCOM1_Handler() {
        if (SpiHw::isr()) { brio::post<SpiBus>(brio::TransferDone{brio::spi_ok}); }
    }

A client answering a transaction - its answers ready, the select still
high:

    Peer::init(clock, {.preload = true});
    const auto r = Peer::answer(std::span<const uint8_t>(frame), spins);
    // r.end == Peer::RunEnd::closed: the host raised the select after
    // clocking r.count answers or more; what it sent is not kept.

The same with what the host sends kept, the run's length the shorter of
the two spans:

    const auto r = Peer::exchange(std::span<const uint8_t>(out),
                                  std::span<uint8_t>(in), spins);

## Bench findings

- Two benches carry this driver. The cross-architecture bench: SAM
  SERCOM1 function C (PA16 MOSI, PA17 SS, PA18 MISO, PA19 SCK) against
  an AVR64EA48 peer's SPI0 ALT1 (PE0 MOSI, PE3 SS, PE1 MISO, PE2 SCK)
  plus GND, both boards at 5 V. And the SAM-SAM bench (both boards'
  PA16..PA19 straight through plus GND). On either desk the same wires
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
  12 MHz reads 1036 and 24 MHz 1680: there the frame (32 and 16 core
  cycles) is shorter than the polled loop's turn, so the instrument
  does not time them (below).
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
- The kernel letter: four requests queued in one dispatch come back
  through their own ReplyTo in order; an over-full arbiter answers
  bus_rejected immediately; an idle bus votes ok on PrepareSleep, a busy
  one refuses. `util/spi_bus.hpp` and `util/bus_master.hpp` arbitrate
  this engine as written: nothing above the contract is target-specific.
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
  eight with the core clock channel disconnected comes back spi_stalled
  after 83 ms - 248000 turns of the flag wait, 16 core cycles a turn -
  the select raised and the receiver empty, and the engine runs the next
  request once the channel is back. An early TXC: 64 characters at 1 MHz
  in the transmit-only shape under a 60 us handler about 120 us in -
  TXC stood at the handler's end, the shifter run dry under it, the next
  write cleared it (32.8.6) and the tail waited for the last character's:
  the peer's receive run counted 64 exact and ended complete, the select
  held to the last. A lost interrupt: a timed `SpiBus` (20 ms) over the
  loop-back engine, SERCOM1's NVIC line taken away once the request was
  on the wire - spi_timeout at 20 ms on the arbiter's clock, `recover()`
  giving the same bus back, the next request ok, no stale completion.
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

- `bench_samc` letter e, THE HOST ABOVE THE WIRE: the host on SERCOM1
  with MISO floating (the time is the wire's), the best of 8, the instrument's figures of the same run
  being ruler 65, stamp 151 (56 of it charged to a handler), interval
  82. The polled loop (`spi.poll` the write shape, `spi.poll.rx` the
  receive one, 256 frames): at 3 MHz the wire's, x = 1.01 both shapes
  (it was 1.45 and 1.54 with one character in flight); at 12 MHz the
  write shape 34 cycles a character against the wire's 32, x = 1.06
  (was 3.06), the receive shape 43, x = 1.33 (was 3.43). The data
  sheet's own sequence written bare over the registers in a scratch
  program on the same board - no library exists for this family - is the
  oracle: a character ahead, reading every answer back, it reads x =
  1.14 at 12 MHz (36.6 cycles a character, 14 instructions a turn) and
  1.00 at 3; the textbook loop with one in flight 2.04 and 1.26. THE
  CORTEX-M0+ AT 48 MHz BEHIND TWO WAIT STATES OF FLASH CANNOT TURN A
  TWO-FLAG POLL AROUND INSIDE A 32-CYCLE CHARACTER, whatever the driver:
  the write shape beats the bare loop because it polls ONE flag; what
  separates brio's receive loop from the bare one is the flash (the
  same loops under `InRam` read 1.05 and 1.21: 33.6 and 38.9 cycles a
  character against 33.7 and 42.4), not the budget - a register
  count-down against a stack slot moved nothing.
  STATUS.BUFOVF after every polled run: never. The pump (`spi.pump`, tx and rx): one interrupt a character,
  252 cycles of `isr` an interrupt with the stamp pair inside (196 the
  handler's body), BUFOVF never - and HANDLER-BOUND at both rates, 391
  cycles of CPU a character whatever the clock: with two in flight the
  handler's exit finds the next character already back and re-enters at
  once, so a pumped transaction at a rate where a character is shorter
  than the handler is a polled one with the interrupt's price added, the
  thread starved until it ends (`launch` reads the whole transaction at
  12 MHz: start() got the core back when it was over), and so is every
  interrupt of the pump's priority with a higher exception number - a
  periodic one loses ticks: under the preemptive kernel a 32-frame
  window at 3 or 4 MHz held TC4's 97 us overflow off 10068 to 10485
  cycles, more than two of its periods, one tick merged into the next a
  window when the preempting dispatch spins 3000 cycles and 2 to 16 in
  40 windows when it spins 300, against 5338 to 5341 at 2 MHz, where
  the handler is shorter than the character
  ([platform.md](platform.md), "The drivers under the preemptive
  kernel"). The pump is for rates where a character outlasts the
  handler; above them a program wants the polled loop. WHERE THE PUMP RUNS FROM (an entry, a character
  of CPU, at 12 MHz): 252 and 391 from the flash; 223 and 356 with the
  host `InRam` behind its vector in the flash (242 and 386 in that
  build's flash image: the same source moves some ten cycles with where
  it sits); 162 and 261 with the vector placed in `.fast_text` and the
  host `InFlash`, the entry inline in it; 180 and 270 with both - the
  entry a call where it was inline. THE FIXED COST OF A REQUEST
  (`spi.req`: a polled request of 1, 3 and 16 bytes with a command byte,
  the D/C scripted on PB23 and the select on PA17 - two real pads, four
  real edges - at 3 MHz): 523, 558 and 647 cycles above the wire, the
  instrument's 82 inside - the phase boundary
  is the 35 between one byte and three (the last command character read
  back, the D/C, the first data character's own start), and the 89
  from three to sixteen is the transmit-only phase's tail (TXC awaited,
  the receiver drained: a phase longer than the receive buffer pays it,
  one of two characters is read back instead and does not). What it is
  made of, counted in the
  listing and measured with the polled transaction `InRam` (4, 7 and
  23 cycles less, the call through the veneer included, so the flash is
  not the bulk of it): the bench's own request
  built on the stack and the interval, the ~35 instructions of start()
  with the one-word compare, the four pin edges inline through
  `PinRef::set/clear` (pin.hpp forces the two verbs inline: each edge
  is the group pointer's load, the null test, the mask's load and one
  OUTSET or OUTCLR store - as calls they cost 49 cycles more on the
  3-byte request, measured), the loop's prologue, and the SERCOM's own
  start and finish latencies at the boundaries.

## Not covered yet

Driver gaps (not built): **a DMA data phase**, on the host or the
client - declined: erratum 1.10.4 ([dmac.md](dmac.md)): the full-duplex
pair hung beside any third active channel, and one channel beside two
others. **Nine-bit frames on the host** - the host's Request is bytes
and its frame eight bits; born with a nine-bit device.

Driver gaps, continued: **the pin edges through the IOBUS alias** -
`PinRef::set/clear` store through the APB bridge, inline (four stores a
request); the single-cycle IOBUS alias (10.1.4) is pin.hpp's and would
take some 12 cycles more off a request's fixed cost, not the hundreds
between it and 300: declined here because the rest of that cost is the
request's shape and the SERCOM's own latencies, stated above. **One
placement for two paths** - the host's tag moves the polled transaction
and the pump's entry together, so a program that places the pump's
vector and wants the polled loops in SRAM pays the pump a call where it
had none (270 cycles a character against 261, measured above); born
with the first program that wants the two apart.

Implemented but not bench-verified:

- The SAM-SAM pairing on the harness's rows (0x3 host, 0x1 client), and
  the samc21 `spi_peer`'s window end and unpulled select against a host
  that clocks the whole answer budget (`test_samc_spi` itself): needs a
  second SAM C21 at 5 V; the AVR64EA48 pairing on these rows is
  measured, both ways.

- The SCK at 12 and 24 MHz ON THE WIRE (BAUD 1 and 0): the polled
  loop's transmit-only turn of some 34 core cycles outlasts their
  frames, so a block's time is the loop's (1036 and 1680 thousandths of
  the wire). A DMA data phase
  would time them - declined, erratum 1.10.4 ([dmac.md](dmac.md)) - or a
  meter on SCK.

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
