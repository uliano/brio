# SERCOM SPI (SAM C21)

Documents of record: SAM C20/C21 data sheet DS60001479M ch. 32 (over the
shared SERCOM ch. 30) and errata DS80000740S 1.17.x. Driver:
`samc21/spi.hpp` over `Sercom<n>`'s instance facts ([sercom.md](sercom.md)
owns the address ladder, the clocks and the NVIC line - one table,
shared by every personality). Family fixture
`test/family_samc21/spi.cpp` + eight negatives; bench suite `test_samc_spi`
(7 letters, two-board: the peer board runs `spi_peer` on an AVR128DB48
or on a second SAM C21, the suite asks ident and does not care -
commanded in band over the bus under test with the `spi_link.hpp` wire
format, one source file serving both architectures).

## What the silicon does

**The two register views are one register set.** The device header
declares `sercom_spim_registers_t` and `sercom_spis_registers_t` with
identical offsets and field positions; the driver uses SPIM for both
roles and the role is CTRLA.MODE and nothing else. That equivalence is
static_asserted field by field at the bottom of spi.hpp, so a device
pack that ever separated them fails the build.

**DOPO is a triple, not a pad.** CTRLA.DOPO's four codes each fix a
whole (DO, SCK, SS) assignment (32.8.1); the three signals cannot be
placed independently. And which signal is which depends on the ROLE
(table 32-2): DO is MOSI on a host and MISO on a client, so one fixed
four-wire harness is a host on row 0x0 and a client on row 0x2 - two
different DOPO codes, not one code with directions flipped. The bench
runs both roles over the same seven wires. The header's enumerator
naming trap: `DOPO_PAD1_Val` names CODE 0x1, which puts DO on PAD[2] -
the same trap TXPO has.

**Enable protection and the three SYNCBUSY bits.** CTRLA, CTRLB, BAUD
and ADDR are enable-protected (32.6.2.1): a store while the instance
runs is DISCARDED in silence - measured (a DIPO flip and a BAUD store
under a running host change nothing). SYNCBUSY has SWRST, ENABLE and
CTRLB (32.8.8); a CTRLB write while SYNCBUSY.CTRLB stands is an APB
error, so every CTRLB write in the driver waits first. ENABLING THE
PERIPHERAL ITSELF RAISES SYNCBUSY.CTRLB (32.8.2's RXEN note): the
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
keeps the bus busy between characters, and keeps exactly TWO characters
in flight - which is the receive buffer's depth: a reader held off for
longer than a character (the polled loop by any handler of the image,
the pump's handler by a late entry) lets k complete into the buffer and
then k + 1, and then the bus STOPS for want of a k + 2 nobody wrote.
Nothing can overflow the receiver that way, at any rate - measured:
STATUS.BUFOVF read after every polled and every pumped run of the
bench's letter e, at 12 and 3 MHz, under the console's and the tick's
interrupts, never set. The textbook loop that writes k + 1 only after
reading k leaves the bus idle for the whole turnaround: a fifth slower
at 3 MHz and twice as slow at 12 (the bare loops, below).

**A client's DATA write needs three SCK cycles to reach the shifter**
(32.6.2.6.2), and those cycles ELAPSE ONLY WHILE SCK RUNS. So an answer
written in the inter-character gap - where a poll loop reacting to RXC
lands - matures mid-character and reaches the wire ONE CHARACTER LATE.
Measured: the host reads the preloaded first answer exactly, then the
whole stream slips by one (11 of 12 mismatched). The working client pump
is ONE AHEAD: preload b0 (CTRLB.PLOADEN puts a write made while SS is
high straight into the shifter, 32.6.3.2), park b1 in DATA at once, and
on every received character write the next-plus-one. So driven, the peer
read this client's 12-character answer stream byte-exact from the first
character.

**A mode change is a CPOL flip on the wire.** Reprogramming a host from
mode 0/1 to mode 2/3 moves SCK's idle level, and that transition is one
extra edge: landed inside an open select window, a selected client
counts it into the character - measured as an exact ONE-BIT SLIP in both
directions, on modes 2 and 3 only. The engine's own `start()` applies
the request's mode BEFORE asserting the request's cs, so an
engine-owned window never sees it; a caller framing CS by hand must call
`SpiHost::prime()` first.

**TXC closes the LAST frame.** "In Host mode, this flag is set when the
data have been shifted out and there are no new data in DATA" (32.8.6) -
measured to mean the END of the last frame and not its start: a
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

**Table 45-59 agrees with the bench, and it is the STRICTER of the
two.** The SPI timing table (simulation values, note 1: not covered by
production test) prices the four directions at VDD > 4.5 V: host
TRANSMISSION tSCK >= 2 x (tMOV 17.1 ns + the device's input setup) -
about 29 MHz against an ideal device, so the 24 MHz TX stream a
client received byte-exact is in spec; host RECEPTION tSCK >= 2 x (tMIS
50.7 ns + the device's output delay) - about 9.9 MHz against an IDEAL
device and 6..7 MHz against a real one, so the loop-back's measured
12 MHz exact EXCEEDS the paper (bench-true on this die at room
temperature, not a design number) and the 24 MHz loop-back failure is
attributed: 101 ns of receiver requirement against a 41 ns period is
the RECEIVER, by the datasheet's own arithmetic. (The 24 MHz stream
and the 12 and 24 MHz loop-back rungs were measured with a DMA-fed data
phase, back to back: facts of the silicon, not paths of this driver.)
A CLIENT's response
path carries tSOSS (~41 ns) plus the far end's setup plus TWO APB
PERIODS (41.7 ns at 48 MHz), which lands the paper ceiling for client
transmission at a few MHz; the client's software one-ahead pump binds
first, at 2..3 MHz (bench findings). Design guidance, then: writes to a
device up to 24 MHz, reads from a real device at 6..8 MHz by the paper
(12 MHz is what this die did), a SAM client's responses a few MHz - and
a fast SPI client is an FPGA's job, the two bus cycles in every
answer being construction, not tuning.

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
are LIVE, cannot be fixed on this side of the wire, and are stated on
`SpiConfig::preload`. 1.17.1 (spurious SSL at enable with SSDE + RXEN)
is REVISION B ONLY - named in the driver precisely because it is the one
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
- **`SpiPads`** - the four pads AND the four pins (the `UartPads`
  shape: the pad side is checked exactly - `spi_dopo_for()` answers
  with the one DOPO row or nothing - and the pin side as far as a header
  can). Role legality is checked IN THE ROLE (`spi_role_probe`), because
  the same harness is legal as a host and illegal as a client on the
  same row.
- **`SpiHost<n, pads, generator>`** - the engine
  `util/spi_bus.hpp` (= `BusMaster`) drives: the shared Request shape
  (cs/dc `PinRef`s, two-phase cmd + full-duplex data, `Borrowed<...,
  Lease::reply>` spans, `ReplyTo<SpiDone>`, per-request BAUD value and
  `SpiMode`, `polled` completion style, and `cs_setup_us` - the
  microseconds between the CS assertion and the first clock, spent
  spinning in start() in main context on samc21/delay.hpp's rate, which
  `rebase()` keeps current so it follows a clock change; measured
  spending 100 asked as 101..103). THE REQUEST IS FORTY BYTES AND NO
  PADDING on this family - the two pin references, the three spans, the
  lengths and the settings packed, the reply capsule last; the field
  names are every target's, the order this family's - and it is LENT
  FOR THE CALL (util/bus_master.hpp): a POLLED request is read through
  the reference and copied nowhere, an asynchronous one is copied once
  into the engine right before its first character goes out, forty bytes
  of inline load-multiple/store-multiple (the compiler expands a struct
  assignment of that size inline where it called memcpy for forty-eight,
  and calls memcpy for a built-in copy of thirty-two: the size is pinned
  by a static_assert for that reason). `apply()` compares the mode and
  the rate as ONE half-word against the applied one and reaches the
  disable/enable pair only on a change, so a run of requests to one
  device costs no register write at all; the SCK ceiling clamps the
  rate as one byte compare. `start()`/`isr()` per the bus_master
  contract, `baud_for()`/`sck_hz()` and the optional bus-wide SCK ceiling
  that `rebase()` re-resolves, `prime()` for callers that frame CS by
  hand. THE POLLED LOOP is the chapter's host sequence a character
  AHEAD: the command phase and any data phase with an rx write k + 1 on
  DRE and read k on RXC (two in flight, the receive buffer's depth - the
  silicon fact above, so no rate threshold and no hazard under the
  image's interrupts); a transmit-only data phase (a null rx: a
  display's pixels) paces on DRE alone, never reads the answers, ends on
  TXC - which closes the LAST character (below) - and drains the receive
  side at its tail. One budget bounds the whole transaction's spins,
  counted down in a register. THE PUMP arms RXC and writes k + 2 from the
  handler that reads k, so two are in flight there too; the first data
  character waits for the last command one to be back (the D/C must
  change on the wire between them), one in flight across that boundary
  alone; the first two characters go out before the interrupt is armed
  (the handler and start() advance the same write index, and at 12 MHz
  the first character is back before start() has written the second).
  Nothing is flushed at start(): every character the pump or a receive
  shape writes is read back by it, and the one shape that overflows the
  receiver - the polled transmit-only phase - drains it at its own tail.
  THERE IS NO DMA DATA PHASE: the polled loop and the pump are the two
  shapes (dmac.md, erratum 1.10.4). `status()` is the completion's word
  - `spi_ok`, or `spi_stalled` (`util/spi_bus.hpp`'s code, one value on
  every family) when a polled transaction's one spin budget ran out with
  a flag never raised. `recover()` is the verb a TIMED SpiBus calls on a
  transaction that never answered (util/bus_master.hpp): CS deasserted
  first, the SERCOM reset and reconfigured to the applied state - the
  wedge it targets is an ISR-style completion that never posts (a lost
  interrupt).
- **`SpiClient<n, pads>`** - the polled surface plus ISR bodies:
  preload, SSDE, address recognition (FORM = 0x2 with AMODE/ADDR),
  `drive_output()` for a dark listener on a shared harness,
  `selected()` as a live pad read (there is no status bit for SS), and
  `frames_ahead` (TWO: how many answers a pump must keep queued ahead
  of the host's clock, the three-SCK-cycle rule's own number - the one
  integer that differs between this family's client pump and the
  other strata's).

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

A client answering a stream (the one-ahead pump):

    Peer::init(clock, {.preload = true});
    Peer::write(b0);            // into the shifter, while SS is high
    Peer::write(b1);            // parked in DATA
    // per received character (poll() or the RXC body): write b(k+2).

## Bench findings

- Two benches carry this driver. The seven-wire cross-architecture
  bench: SAM SERCOM1 function C (PA16 MOSI, PA17 SCK, PA18 SS, PA19
  MISO) against an AVR128DB48 peer's SPI0 ALT1 (PE0-PE3), both boards
  at 5 V. And the SAM-SAM five-wire bench (both boards' PA16..PA19
  straight through plus GND). On either desk the same wires carry a
  board as host (DOPO row 0x0) and as client (row 0x2).
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
  bind at the PEER'S ANSWER RELOAD, and the ladder has two measured
  boundaries. The AVR peer's polled loop: exact to 500 kHz
  always, 1 MHz a coin toss (its 5..9 us polled turnaround against a
  10 us character), well below its CLK_PER/6 electrical ceiling. The
  SAM peer's polled loop (precomputed stream, one-ahead reload): exact
  to 2..3 MHz - and at the first failing rung the peer still hears
  every byte exact, so the boundary is the reload, not the wire.
- The kernel letter: four requests queued in one dispatch come back
  through their own ReplyTo in order; an over-full arbiter answers
  bus_rejected immediately; an idle bus votes ok on PrepareSleep, a busy
  one refuses. `util/spi_bus.hpp` and `util/bus_master.hpp` arbitrate
  this engine as written: nothing above the contract is target-specific.

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
  same loops placed in SRAM read 1.05 and 1.21), not the budget - a
  register count-down against a stack slot moved nothing.
  STATUS.BUFOVF after every polled run: never. The pump (`spi.pump`, tx and rx): one interrupt a character,
  226 cycles of `isr` an interrupt with the stamp pair inside (170 the
  handler's body), BUFOVF never - and HANDLER-BOUND at both rates, 377
  cycles of CPU a character whatever the clock: with two in flight the
  handler's exit finds the next character already back and re-enters at
  once, so a pumped transaction at a rate where a character is shorter
  than the handler is a polled one with the interrupt's price added, the
  thread starved until it ends (`launch` reads the whole transaction at
  12 MHz: start() got the core back when it was over). The pump is for
  rates where a character outlasts the handler; above them a program
  wants the polled loop. THE FIXED COST OF A REQUEST
  (`spi.req`: a polled request of 1, 3 and 16 bytes with a command byte,
  the D/C scripted on PB23 and the select on PA18 - two real pads, four
  real edges - at 3 MHz): 526, 559 and 665 cycles above the wire, the
  instrument's 82 inside - the phase boundary
  is the 33 between one byte and three (the last command character read
  back, the D/C, the first data character's own start), and the 106
  from three to sixteen is the transmit-only phase's tail (TXC awaited,
  the receiver drained: a phase longer than the receive buffer pays it,
  one of two characters is read back instead and does not). What it is
  made of, counted in the
  listing and measured with the polled path placed in SRAM (38 cycles
  less, so the flash is not the bulk of it): the bench's own request
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
request's shape and the SERCOM's own latencies, stated above. **The polled loop in SRAM** - a tenth at 12 MHz, measured;
declined as a default because a placement of some 2 KB in `.ram_text`
is the program's call (platform.md, "A handler in SRAM"), not the driver's.

Implemented but not bench-verified:

- The SCK at 12 and 24 MHz ON THE WIRE (BAUD 1 and 0): the polled
  loop's transmit-only turn of some 34 core cycles outlasts their
  frames, so a block's time is the loop's (1036 and 1680 thousandths of
  the wire). A DMA data phase
  would time them - declined, erratum 1.10.4 ([dmac.md](dmac.md)) - or a
  meter on SCK.

- A polled transaction's spin budget running out (`spi_stalled` in
  `status()`, the receiver flushed, the select raised): no letter stops
  the SERCOM's clock under a transaction. A request started with the
  instance's GCLK channel or CTRLA.ENABLE down would measure it - the
  wall of the budget and the code in the reply.
- The polled transmit-only phase's TXC under a handler longer than a
  character: an early TXC is cleared by the next write (32.8.6) and the
  tail's wait is the last character's by the chapter's words; letter e
  runs under the tick and the console alone, and the write-shape
  numbers are its witness only for that.
- Sleep: RUNSTDBY on silicon, SSDE as a wake source, erratum 1.17.20's
  standby cost - no letter sleeps this bus.
- On silicon: SSDE/SSL, address recognition (FORM = 0x2 - refusals are
  compile-checked, matching is not bench-verified), 9-bit characters
  through a real two-board transfer (loop-back only), IBON = 0's
  travelling overflow flavour.
- Erratum 1.17.3's dummy-first-character (needs a host that RAISES SS
  mid-transmission on purpose; the peer holds it low, correctly).
- The timed-bus path on this wire: `SpiBus`'s per-bus timeout and
  `recover()` are host-tested (deterministic race legs included) and
  compile-proven here, but no SPI wedge has been staged on silicon -
  the I2C letter l is the mechanism's silicon witness (a held wire
  stages it there; here a SERCOM interrupt lost under a running pump
  would).

Stated, not enforced: `SercomPadPin`'s pin-reaches-pad claim is the
caller's (sercom.md's open device-table question, unchanged here).

Open, and outside this driver: THE PEER'S SELECT-WAIT WEDGE,
neutralized but not explained. A peer that opens its exchange window by
spinning on the select READ can enter a persistent state - until its
board is reset, about once in five `z` runs - where that read never
fires while its SPI HARDWARE demonstrably shifts: the host reads the
preloads and then the echo, the software window reads nothing. On the
wire the select is real (driven low over SWD, and the wedged peer's own
status reads it LOW), so the wedge lives between the pad and that wait.
The exchange loop therefore polls RXC directly - a byte can only arrive
while selected, and apply_cfg has just cleared the buffers - and
samples the select purely as TELEMETRY the Report carries
(aux1..aux3), which removes the symptom without explaining it. What
the state is remains unhunted, a question on the AVR side of the link
(the peer plus its pin read path) with the telemetry in place to catch
it in the act.
