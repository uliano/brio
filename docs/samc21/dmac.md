# DMAC (SAM C21)

This document is for a programmer who wants a serial port's or an SPI
bus's bytes off the CPU on the SAM C21, and needs to know how far this
chip's DMA controller can be trusted. The driver offers two engines,
`DmaTxEngine<ch>` and `DmaRxEngine<ch>`, each a channel that moves a
run of bytes between memory and one peripheral's DATA register, one
byte per request of the peripheral, with no interrupt per byte. A
program never calls them: it names them in a transport's template
arguments - `samc21/sercom.hpp`'s `Uart` ([sercom.md](sercom.md)),
`samc21/spi.hpp`'s `SpiHost` ([spi.md](spi.md)) and `samc21/i2c.hpp`'s
`I2cHost`, the transmit engine alone ([i2c.md](i2c.md)) - and binds the
DMAC's one interrupt vector to those transports. The prerequisite that decides
correctness is a budget: AT MOST TWO CHANNELS IN AN IMAGE, because this
silicon corrupts its transfers when three run at once (erratum 1.10.4,
"The rule" below), and each of the two has one owner for the whole image,
which the compiler checks.

The smallest use is a console whose transmitter runs on the engine, the
receiver on its interrupt:

```cpp
#include "samc21/dmac.hpp"
#include "samc21/sercom.hpp"

using Serial = brio::Uart<5, console_pads, 64, 256, brio::DmaTxEngine<0>>;

extern "C" void SERCOM5_Handler() {     // the receiver
    if (Serial::isr()) { brio::post<SerialLines>(brio::RxActivity{}); }
}
extern "C" void DMAC_Handler() { (void)Serial::dma_isr(); }   // the transmit blocks
```

Line by line: `Serial` is the console's Uart (instance 5, the pads of
sercom.md's console, a 64-byte receive ring and a 256-byte transmit
ring) with the transmit engine on channel 0 in its transmit slot; the
receive slot stays at its default, no engine. The program reaches the
engine by including `samc21/dmac.hpp` beside `samc21/sercom.hpp`. The
SERCOM's handler stays the receiver's, posting `RxActivity` to the
program's line-assembling active object (an active object is a
component of a brio program with its own event queue, served one event
at a time by the kernel, [docs/learn/introduction.md](../learn/introduction.md)).
The DMAC's handler calls `dma_isr()`, the body that hears a block's
end, releases its bytes from the ring and starts the next run; its
answer is the receive edge, which a transport with no receive engine
never raises, so the cast discards it. The program calls the engine
nowhere else: `Serial::init()` arms it and `print` feeds it through the
ring.

What decides correctness even there. The budget is the board file's to
spend: the engines named across the whole image hold two channels at
most, numbered 0 and 1, and a third number does not compile. Each channel
belongs to one peripheral request for the whole image - here SERCOM5's
transmit request - and the transport that names an engine claims it when
the program is compiled: a second transport of another SERCOM naming the
same channel, in turn or at once, does not compile ("One owner a
channel", below). Both vectors are the program's to bind: an
engine arms its interrupts and opens the DMAC's NVIC line at `init()`,
and a `DMAC_Handler` left undefined is a weak alias of the crt's silent
spin, where the program stops at the end of its first block. Every
transport holding a channel is called from that one vector, because the
line stays up while any armed flag stands and each transport takes its
own channels' flags and nobody else's. The transport's `init()` answers
false when the DMAC or a channel would not reset within its bounded
wait - one false among the transport's own reasons, the answer not
saying which. The run a
block carries is the transport's own storage (its ring, or the request's
buffer), lent to the channel until the block's end. And a standby
entered with a block in flight is outside what this stratum has run
("Not covered yet").

## Documents of record

SAM C20/C21 data sheet DS60001479M ch. 25, and errata DS80000740S items
1.10.1..1.10.4, whose matrix is read below for this chip (E/G/J family,
silicon rev F). Driver: `samc21/dmac.hpp` - `Dmac`, `DmaChannel<ch>`,
`DmaTxEngine<ch>`, `DmaRxEngine<ch>`; their users `samc21/sercom.hpp`'s
`Uart`, `samc21/spi.hpp`'s `SpiHost` and `samc21/i2c.hpp`'s `I2cHost`.
Family fixtures
`test/family_samc21/sercom.cpp`, `spi.cpp` and `i2c.cpp` with the
negatives `dmac_channel_past_budget.cpp`, `dma_two_owners_on_one_slot.cpp`,
`dma_one_request_on_two_slots.cpp`, `dma_third_engine_in_an_image.cpp`,
`uart_receive_engine_alone.cpp`, `uart_engines_one_channel.cpp`,
`spi_receive_engine_alone.cpp`, `spi_engines_one_channel.cpp` and
`i2c_engine_slot_receive_engine.cpp` under `brio check samc21`. The bench
letters (a letter is one command key of a suite's console and the group
of verdicts that key runs): `test_samc_uart` letters d (the engine's
level), u (the Uart's pair at three rates), w (what a release leaves the
same transport's next `init()`), 2 and 3 (two shapes for minutes);
`test_samc_spi` letters l to r (the SPI host's engines, the shapes, a
channel disabled mid-block, the write's rate); `test_samc_i2c` letter s
(the I2C host's writes beside the console's engine for ten minutes). A
suite whose letters give a channel to two requests is built as one image
per arrangement (its `// build: groups` line), each image's channels with
one owner.

## What the silicon does

**Twelve channels behind one register set.** CHCTRLA, CHCTRLB, the
interrupt registers and CHSTATUS talk to whichever channel CHID names
(25.8.17), and every descriptor lives in SRAM: a first-descriptor section
at BASEADDR and a write-back section at WRBADDR, one 16-byte entry per
channel (25.6.2.3). One vector serves all twelve; INTSTATUS has one bit
a channel, set while an armed flag of that channel stands (25.8.11), and
CHINTFLAG behind CHID holds the flags.

**The write-back is the transfer, not a report.** "For an ongoing block
transfer, the descriptor will be fetched from the write-back memory
section" (25.6.2.6): when a block is interrupted by the arbitration or
resumed, the controller continues from what it wrote back. Whatever
corrupts that entry corrupts the transfer.

**No hardware circular mode.** 25.6.3.1 offers a self-linked descriptor
and nothing else; a channel that repeats does so from linked descriptors
or from the CPU.

**Four arbitration levels, each switched on by hand.** A channel takes
one of four levels (CHCTRLB.LVL), and while every level is enabled a
higher number wins; within a level the lower channel number wins, or a
round robin does (PRICTRL0.RRLVLENx); a level is enabled by CTRL.LVLENx,
and a channel at a level not enabled is invisible to the arbiter
(25.6.2.4; measured). The DMAC is also a host of the SRAM with a
quality of service of its own (QOSCTRL, 25.8.7; 10.4.3), a separate knob
from the channel's level. The engines arm at the rule of
[../design/dma.md](../design/dma.md): a receive channel at level 3, a
transmit one at level 2, both levels enabled. Within the budget the
level protects nothing against the erratum (two channels measured clean
at every setting); it ranks a receive above a transmit because a late
receive loses data and a late transmit only waits, and two transmit
channels by their number, the lower first. QOSCTRL stays at its reset
value.

**A request is the trigger line's level, and the channel keeps none.**
CHCTRLB.TRIGSRC names one peripheral line, and a SERCOM's transmit line
is a level - "set when the transmit buffer (TX DATA) is empty", "cleared
when DATA is written" (31.6.4.1). A request standing when the channel is
enabled, or when its trigger is selected onto it, is served at once: the
enable of a block fires its first beat by itself. A request that came
and went while the channel was disabled is not kept - a disabled channel
leaves the queue of pending ones and its PEND is cleared (25.6.2.4,
25.8.23) - measured below.

What the engines rely on beyond that - the end address of an
incrementing side, the disable that drains before it clears, the silent
SWRST, BLOCKACT, the live count in ACTIVE - is written in the driver's
header, each with its section.

## Erratum 1.10.4, and the revision matrix

DS80000740S 1.10.4, "Concurrent channels triggers": "When using
concurrent channels triggers, the DMAC write-back descriptors may get
corrupted." Workaround: "Multiple transfers must only be sequenced using
linked descriptors on a single channel".

The matrix, read by ROW (an X marks an affected revision):

| item | E/G/J | N |
|---|---|---|
| 1.10.1 CRCDATAIN writes | B | - |
| 1.10.2 Fetch error (linked descriptors) | B, C, D | E, F |
| 1.10.3 Enabling channels (linked descriptors) | B, C, D | E, F |
| 1.10.4 Concurrent channels triggers | E, F, H | - |

On the E/G/J die the two linked-descriptor items leave at revision E,
where 1.10.4 arrives; the N die lists 1.10.2 and 1.10.3 at E and F and
not 1.10.4. The bench chip is an ATSAMC21J18A at revision F: 1.10.4 is
live and the other three are not. THE TRAP: for 1.10.2 and 1.10.3 it is
the N row that carries marks under E and F - read the row, not the
column.

## The rule

What follows was measured on one ATSAMC21J18A at silicon revision F, so
every bound is one die's; the erratum lists revisions E, F and H, and
the rule is written for them.

1. **An image runs at most two DMA channels, of any direction and any
   owner.** Two active channels never faulted in any shape measured;
   three or more faulted in every shape, whoever owned them, under
   every arbitration and quality-of-service setting the controller
   offers.
2. **No channel is suspended, resumed or disabled mid-block while
   another channel runs.** A software suspend of a running channel
   (CHCTRLB.CMD = SUSPEND, then RESUME) corrupts the OTHER channel's live
   descriptor within a second, whichever of the two is suspended; a
   disable of a channel with a beat in flight loses the OTHER channel's
   pending trigger in 0.7 % of disables (the block stalls enabled, its
   request line high and never raised again). Progress is read with no
   suspend (the live count or the write-back as it stands) or at the
   block's completion edge; a channel is disabled only at its block's
   completion or when no other channel runs.
3. **The budget is spent by the board file, and the compiler keeps it.**
   Each engine is one channel, a full-duplex pair two, armed at its
   transport's `init()` and held until `release()`. An engine names
   channel 0 or 1 and nothing else, so no image can make a third channel
   run; and each channel has one owner for the whole image, the request
   its transport serves, so a third engined transport of an image names a
   channel another request owns and does not compile. A channel never
   changes hands.
4. **Nothing else of the controller is offered:** no linked descriptors,
   no event-triggered channel, no memory-to-memory copy, no converter or
   timer stream, no CRC through the data port. Each is declined in "Not
   covered yet" with the measurement or the erratum that declines it.

How the driver keeps it. Sentence 1 is structure: two section entries,
two channel numbers, `neg/dmac_channel_past_budget.cpp`. Sentence 3 is
the ownership claim of [../design/dma.md](../design/dma.md), "The
ownership rule", made when the program is compiled ("One owner a
channel", below): nothing of it runs, and the engines keep no record of
an owner. Sentence 2 is the absence of a suspend verb anywhere in the stratum, a
receive engine that comes only with its transmit engine (so its owner
knows no other channel runs when it lets the receive channel go:
`neg/uart_receive_engine_alone.cpp`), a `release()` that waits for the
transmit block in flight to end, and the paths that can still break it,
each a recovery or an early end, stated in "The rule's exceptions" below.

## One owner a channel

The ownership rule of [../design/dma.md](../design/dma.md) ("The
ownership rule") on this controller. A slot is a channel, 0 or 1, keyed
`DmaSlotId<0, ch>` (the DMAC numbers no controller). Its owner is the
request it serves: a SERCOM's transmit or receive line, keyed
`DmaOwner<DmaPeriph::sercom, n, DmaSignal::tx>` or `rx` whichever
personality the SERCOM has - a Uart, an SPI host and an I2C host on one
SERCOM raise the same transmit request. The claim is made in the class
body of each transport that names an engine (`Uart`, `SpiHost`,
`I2cHost`), through `samc21/sercom.hpp`'s `sercom_dma_claimed()`, so it
counts wherever the program uses the transport type, and it binds both
ways:

- a second request on a channel does not compile - the console's Uart on
  SERCOM5 and a display's SPI host on SERCOM1 both naming channel 0:
  `ambiguating new declaration of 'constexpr brio::DmaOwner<brio::DmaPeriph::sercom, 1, brio::DmaSignal::tx> brio::dma_owner_of(DmaSlotId<0, 0>)'`,
  its note naming SERCOM5's owner (`neg/dma_two_owners_on_one_slot.cpp`);
- one request on both channels does not compile either - two transport
  types of SERCOM0 naming its transmit engine on channel 0 and on
  channel 1: `ambiguating new declaration of 'constexpr brio::DmaSlotId<0, 1> brio::dma_slot_of(DmaOwner<brio::DmaPeriph::sercom, 0, brio::DmaSignal::tx>)'`
  against `DmaSlotId<0, 0>` (`neg/dma_one_request_on_two_slots.cpp`);
- and with two channel numbers, a third engined transport is always one
  of the two (`neg/dma_third_engine_in_an_image.cpp`): the budget of two
  is the compiler's, whole.

What passes is one request served by several transport types: a Uart
and an SPI host on SERCOM1 naming the same transmit channel, or two SPI
host types of one SERCOM (a pair and a write-only host), take that
channel in turn, each releasing before the next one's `init()` - one
owner, the request, whatever the transport. A claim adds no byte to the
image and no cycle (util/dma_owner.hpp).

What the map leaves to choose is nothing but the count. Any trigger
reaches any channel (CHCTRLB.TRIGSRC, 25.8.19), so no two requests
contend for a channel of the silicon's; the budget alone decides what an
image may hold, and it forces these choices:

| the image wants | what it gets on DMA | why |
|---|---|---|
| a console's transmit and receive both on DMA | the console's pair, and no other engine | the pair is the whole budget |
| a duplex UART link and a full-duplex SPI on DMA | one of the two | each pair is the whole budget |
| a console's transmit and an SPI host that reads | the SPI pair alone, the console on its interrupt | an SPI host reads only on its pair |
| a console's transmit beside a display's writes | both: console TX on one channel, SPI TX on the other | the display's shape, two channels |
| a console's transmit beside an I2C device's writes | both: console TX and I2C TX | two channels |
| a third DMA transport of any kind | none: it runs on its interrupt or polled | the budget |

The unavoidable groups, then, are every set of three engines on this
family's three parts alike (the E, G and J packages share the DMAC and
the erratum): erratum 1.10.4 (DS80000740S; live on E/G/J at revisions E,
F and H) and the measurements below make the budget two, so any two
engines spend it, and a console with both of its engines on DMA fills it
alone.

## What the rule rests on

| shape | what it serves | measured | faults | bound (95 %) |
|---|---|---:|---:|---|
| Uart TX + SPI TX | a logging console beside a display's pixels | 10,911,165 SPI requests beside 2.4 M console and dense blocks, every arrangement of channel numbers, levels and round robin, display refresh at 12 and 24 MHz | 0 | < 2.8e-7 per request |
| Uart TX + Uart RX (one SERCOM, both engines) | a duplex data link, bulk both ways | 5,414 windows / 64 MB at 115200 and 1 Mbaud; 3,032,700 raw block pairs at 3 Mbaud, 1 Mbaud and 115200; 10 host-echo runs with drifting phase | 0 | < 1.0e-6 per block pair |
| SPI TX + SPI RX (one SERCOM, both engines) | full-duplex SPI, reads included | 24,817,271 requests, 5..256 frames at 1.5..12 MHz, both channel orders | 0 | < 1.3e-7 per request |
| Uart TX + SPI RX-only | a logging console beside an SPI read whose transmit side the CPU writes | 12,294,674 requests beside 250,875 console blocks | 0 | < 2.5e-7 per request |
| Uart TX + I2C TX | a logging console beside a display over I2C, write-only | 1,461,242 I2C writes (188.7 MB, 1..255 bytes) at 400 kHz and 1 MHz beside 355,316 console blocks at ~90 % of the line | 0 | < 2.1e-6 per write |
| one TX channel + a memory copy or a timer-paced loop | the control shapes | 13.6 M requests | 0 | < 2.3e-7 per request |

A bound is the one-sided 95 % bound on a rate with no event seen, 3
over the count. The measurements of shapes this driver does not build -
Uart TX + SPI RX-only, a copy or a loop beside a channel - stand as the
evidence for sentence 1; the driver offers Uart TX + SPI TX, the Uart
pair, the SPI pair, Uart TX + I2C TX and the console alone (sercom.md,
spi.md, i2c.md).

The shapes the rule refuses:

| shape | faults | what it looked like |
|---|---:|---|
| THREE active channels, any owners (SPI pair + copy or loop; SPI TX + copy + loop; copy + two loops, no SPI at all; SPI RX-only + copy + loop) | 4..17 % of requests hung within the first second, silent completions with bytes missing | the victim enabled with CHSTATUS 0, its write-back holding a HIGHER-numbered channel's BTCTRL and SRCADDR over its own DSTADDR; the highest-numbered active channel the one never hit |
| three channels with distinct levels, or round robin | 16 % (three times the control) | the highest-level channel a victim too; 202,425 words of RAM written below a loop's destination in two minutes; one hard fault |
| three channels with QOSCTRL high, CPU QoS low, or the CPU in WFI | 5..6 % | no knob cures it |
| two channels, one SUSPENDED and resumed while the other runs (a suspend every 1 ms or 100 us on either channel) | first fault 12..736 ms in every cell; 0.02 % (115200) to 1.4 % (3 Mbaud) of suspends; the board dead at 3 Mbaud with a 100 us cadence | the OTHER channel's write-back holding the suspended channel's BTCTRL, BTCNT and SRCADDR; with DSTINC set the transfer then writes the SERCOM's registers below DATA (INTENSET scribbled 69 times in one cell), blocks abandoned and re-sent (the far end receiving up to 900 KB more than sent), wedges |
| two channels, one DISABLED mid-block while the other runs (a Uart's receive channel taken from under a running transmit channel) | 167 of 23,813 disables (0.70 %; 0.38 % at 3 Mbaud, 1.12 % at 1 Mbaud) | the running channel left enabled with its OWN descriptor in its write-back, CHSTATUS 0, ACTIVE still naming it with ABUSY, DRE and TXC up: a lost trigger, not a foreign descriptor - DRE is a level the controller takes on its rise, so a request dropped while it stays high is never raised again; the block stalls until a dead-block predicate abandons and re-sends it (242..1,024 bytes duplicated each time) |

The rate per suspend grows with the line rate - the chance that the
other channel has a beat in flight during the suspend - and not with the
cadence. The disable's rate runs the other way (higher at 1 Mbaud than
at 3 Mbaud) and is not explained; the same disable with no other channel
running never faulted (15,381 disables), and at the block's completion
edge never faulted (52,943).

The driver as it stands, held on the bench beside its partner, every
class of fault counted (requests not ended `spi_ok`, bytes wrong at
either end, blocks abandoned, console lines lost or broken):

| shape | channels | held | what ran | faults |
|---|---|---|---|---:|
| the display's: the SPI host's transmit engine beside the console's | SPI TX on 1, console TX on 0 | 600 s | 1357 windows of 256 frames at 2 MHz exact at an AVR64EA48 client, 5428 blocks of 1024 at 12 MHz on the loop-back, 600 console lines (`test_samc_spi` letter o) | 0 |
| the SPI pair, the console on its interrupt | SPI TX on 0, RX on 1 | 600 s | 15,790 windows of 16 frames both ways at 2 MHz with the client, 63,160 reads of 1024 at 12 MHz on the loop-back, every byte checked (letter p) | 0 |
| the Uart pair | Uart TX on 0, RX on 1 | 600 s | 59,999,732 bytes at 1 Mbaud on a loop through the pad, every byte checked, beside the console on its interrupt (`test_samc_uart` letter 2) | 0 |
| a console alone | Uart TX on 0 | 600 s | 87,863 marked lines read whole by the host (letter 3) | 0 |
| the I2C host's writes beside the console's | I2C TX on 1, console TX on 0 | 600 s | 135,131 writes of 1..254 bytes at 400 kHz and 1 MHz exact at an AVR64EA48 client, 1195 of them NACKed mid-block and abandoned, 135,131 console lines read whole by the host (`test_samc_i2c` letter s) | 0 |

## The rule's exceptions

Five paths of the driver can still disable a channel mid-block beside a
running one, each a recovery from something that already went wrong or a
block the wire ended early:

- **`SpiHost::recover()` with a request on the engines**, the timed
  arbiter's verb on a request that never answered: its channels are put
  away mid-block, and on the transmit engine alone another transport's
  channel may be running beside them (the pair holds both channels, so
  there no other runs). Measured with the case that path makes - the SPI
  host's transmit channel disabled a pseudo-random 0..150 us into a
  256-frame block beside a Uart's transmit engine streaming at 1 Mbaud,
  its receiver judging every byte (`test_samc_spi` letter q): no lost
  trigger, no byte lost or repeated, no block abandoned, in 25,000
  disables as written and 25,000 with the SPI's request withdrawn first
  (below 1.2e-4 a disable at 95 % in each of the two, against the 0.7 % a receive channel
  disabled under a running transmit channel lost). What can happen beyond
  that bound is the measured kind: the running channel's pending trigger
  lost, its block stalled until the Uart's dead-block repair abandons it
  at its next refused byte and re-sends what the block had not
  acknowledged (242 to 1,024 bytes repeated on the far end in the
  measurement that found it).
- **The Uart's dead-block repair** abandons a transmit block whose
  channel waits enabled with nothing pending while its trigger stands:
  a channel that lost its trigger, with no beat in flight. It disables
  that channel beside another transport's running one; it runs only after
  such a loss, the loss above being its one known cause.
- **The SPI host's lost trigger, on the transmit engine alone**: TXC and
  DRE up in the SERCOM's vector with the host's transmit channel enabled
  and nothing pending end the request `spi_dma_fault`, and the host puts
  that channel away while the other channel - a console's transmit
  engine, in the display's shape - may be running. No beat is in flight,
  so it is the dead-block repair's act, and like it the path runs only
  after a lost trigger.
- **An I2C write the wire ended before its block did**: a data NACK, a
  lost arbitration or a bus error while the I2C host's transmit block
  still has beats (and its timed arbiter's `recover()` on a write that
  never answered). The SERCOM raises no transmit request after the end,
  so the channel waits enabled with no beat in flight, and the host
  abandons it - the act of the Uart's dead-block repair. Measured beside
  the console's transmit channel running (`test_samc_i2c` letter s): 1195
  such abandonments, the console's channel enabled at 93 % of the writes'
  starts, no console block lost, stalled or repeated and every line read
  whole by the host (below 2.6e-3 a disable at 95 %).
- **A `release()` whose transmit block outlives its bound** (the block's
  wire time): the channel disabled anyway, the transmitter having
  stopped.

A transfer error is not among them: it ends its block on the silicon's
own disable. The SPI host's fault path then puts its own channels away -
on the pair the other channel of the pair, the two holding the whole
budget so that no third runs; on the transmit engine alone the channel
the silicon has already disabled.

## Types and verbs

- **`Dmac`** - the block's public face: `irq()`, its NVIC line; `budget`,
  2, the channel numbers an engine may name. The first channel armed
  brings the block up (its AHB clock, SWRST, the two sections, levels 2
  and 3 enabled), the last one released takes it down; a channel between
  them touches itself alone.
- **`sercom_dma_claimed<Engine, n, signal>()`** (`samc21/sercom.hpp`) -
  the claim a transport makes in its class body for each engine slot:
  the engine's channel to SERCOM `n`'s `signal` request ("One owner a
  channel"); true, and nothing claimed, for an empty slot.
- **`DmaChannel<ch>`** - one channel, what both engines do on it:
  `arm(trigger, level, completion)`, `release()`, `take()` (the
  channel reset and configured whatever its state), `enabled()`,
  `waiting()` (enabled, nothing pending, no beat moving; when its owner
  has also read the channel's trigger standing, that is a dead block),
  `priority()`
  (the level read back, nothing when CTRL leaves it disabled),
  `take_interrupt()` (the channel's flags taken and cleared, one load of
  INTSTATUS when none stands), `observe()` (the beats still to move, read
  with no suspend: ACTIVE's live count while the channel holds the active
  registers, the write-back otherwise, a write-back that disagrees with
  the slot refused and counted in `violations()`). Every access to the
  channel's registers from main context is a CHID select and its use
  under the platform's critical section and nothing more; a vector's
  needs none, a handler here running to completion.
- **`DmaTxEngine<ch>`** - a channel draining bytes into a peripheral's
  DATA, at level 2: `arm(data, trigger, completion)`, `release()`; for
  an owner that starts blocks from two contexts,
  `reserve()` (a test-and-set its caller masks) then `launch(run)` (three
  stores into the slot and the enable, false for an empty run or one past
  65535 bytes) or `cancel()`; `start(run)` both; `start_run(p, count)` and
  `start_fixed(cell, count)` for an owner that starts from one context and
  has checked the count (the second the SPI host's dummies, the source
  held still); `complete()` (the bytes the block carried), `in_flight()`,
  `busy()`, `running()`, `waiting()`, `abandon()` (the channel reset and
  re-armed, the block's tail lost, counted in `faults()`).
- **`DmaRxEngine<ch>`** - a channel filling bytes from a peripheral's
  DATA, at level 3: `arm`, `release`, `prepare(run, count)` then
  `launch()` (the enable alone, for an owner whose critical section
  covers it),
  `start(run, count)`, `idle()`, `take_landed()` (what landed since the
  last take, through `observe()`), `complete()` (what the block's end
  left untaken), `abandon()`.
- **`DmaCompletion`** - what a block's end does, chosen at `arm()`:
  `interrupt` (TCMPL armed: the Uart's engines, and the I2C host's,
  whose block's end re-arms the MB its write ends on) or `silent` (TERR
  alone: the SPI host's, which ends a request on the SERCOM's TXC).

## How to use it

The console's transmitter on channel 0, the receiver on its interrupt -
the smallest use above. A duplex link on both engines, the pair holding
the whole budget, its receive side published at each completion (half
the receive ring) and its tail on `harvest()`:

```cpp
using Link = brio::Uart<1, link_pads, 1024, 1024, brio::DmaTxEngine<0>, brio::DmaRxEngine<1>>;

extern "C" void DMAC_Handler() {
    if (Link::dma_isr()) { brio::post<LinkLines>(brio::RxActivity{}); }
}
// from the line-reading active object, at its own cadence (a time event):
//   if (Link::harvest()) { ... the bytes that arrived since the last look ... }
```

A display's pixels on the transmit engine alone (channel 1) beside the
console's transmitter on channel 0 - the budget spent across two
transports, each channel with its one owner, one vector serving both:

```cpp
using Serial = brio::Uart<5, console_pads, 64, 256, brio::DmaTxEngine<0>>;
using SpiHw = brio::SpiHost<1, display_pads, 0, brio::DmaTxEngine<1>>;

extern "C" void DMAC_Handler() {
    (void)Serial::dma_isr();
    if (SpiHw::dma_isr()) { brio::post<SpiArb>(brio::TransferDone{SpiHw::status()}); }
}
extern "C" void SERCOM1_Handler() {
    if (SpiHw::dma_isr()) { brio::post<SpiArb>(brio::TransferDone{SpiHw::status()}); }
}
```

An SPI host that reads on the engines takes both channels; the console
beside it stays on its interrupt (spi.md, "How to use"). An I2C host's
writes on the transmit engine beside the console's take the same two
channels the display's pixels take above, its DMAC body beside the
console's in the one vector (i2c.md, "How to use", the second block). Two
transport types of one SERCOM may take its channel in turn - one
request, one owner - each releasing before the next one's `init()`; a
transport of another SERCOM naming that channel does not compile. An
excerpt, the pads each SERCOM1's and `clock` the program's clock tag:

```cpp
using Link = brio::Uart<1, link_pads, 64, 512, brio::DmaTxEngine<0>>;  // SERCOM1's transmit request
using Bus = brio::SpiHost<1, spi_pads, 0, brio::DmaTxEngine<0>>;       // the same request: one owner

Link::release();                    // the channel stopped
const bool up = Bus::init(clock);   // armed again for the same owner; false: the bus is not up
```

## Bench findings

- **The budget, by the compiler** (`brio check samc21`): a second owner
  on a channel, one request on both channels and a third engined
  transport each fail to compile on the three packages, the error the
  claim's (the three `neg/dma_*.cpp` above); the same transports with the
  channels given one owner each compile, and every image of the stratum
  links with its channels claimed.
- **The level read back** from an armed channel, with that level enabled
  in CTRL: 2 for a transmit engine (`test_samc_uart` letter d, the
  console's engine); a channel at a level not enabled is invisible to the
  arbiter.
- **A released requester** (`test_samc_uart` letter w, no wire: the
  Uart's transmit engine on SERCOM1's loop through the pad, PA18, at
  1 Mbaud): the engine's trigger standing - its last block over, the
  channel disabled by the block's end (25.6.2.6), DRE up behind it - then
  the transport released three ways, then the same Uart initialized
  again over the same channel, its one owner.
  - **The channel keeps no request.** DATA filled by hand behind the
    engine's last block - the first character straight to the idle
    shifter, so DRE fell and rose again with the channel disabled, the
    second left in DATA, DRE low - and a block of one started: the loop
    read 0xC1, 0xC2, then the block's 0xB1, in order.
  - **A disable withdraws the request.** Disabled and gated, the instance
    read through its bus clock alone shows INTFLAG 0x00 with CTRLA's
    configuration standing (0x40000004).
  - **A gated clock holds a request left standing.** Gated with ENABLE
    set: INTFLAG reads 0x03 (DRE and TXC) behind the gate. A channel
    bound by hand to that trigger took ONE stray beat - the first byte of
    its block - and waited, enabled, for a rise that cannot come; and the
    same channel bound to no trigger (TRIGSRC 0) moved nothing, the
    engine's `arm()` resetting the channel (25.6.2.2).
  - **The reset leaves nothing.** `release()` resets the SERCOM before
    the gate (CTRLA.SWRST, 31.6.2.2, 31.8.1): INTFLAG and CTRLA read zero
    behind it.
  - After each of the three, the same Uart's next `init()` sent its first
    256 bytes whole and in order, no stray beat and no fault: a
    transport's own `init()` resets its SERCOM before it arms its
    engines.

  So the SAM C21 gives the STM32F4's answer of
  [../design/dma.md](../design/dma.md) at the peripheral - a disable
  withdraws a raised request, a gated clock does not - and keeps nothing
  in the channel. The stray reaches only a binding of the SAME trigger,
  which under the ownership rule is the same transport's next `init()`,
  and that `init()` starts from a reset SERCOM.
- **A bus error** disables its channel and sets TERR (25.6.2.8): a read
  on the SPI pair whose transmit run crosses the SRAM's end (0x20008000
  on the J18A) ended `spi_dma_fault` in every run of `test_samc_spi`
  letter n, the core taking no fault, and the next request exact (spi.md).
- **The write on one channel against the wire** (`test_samc_spi` letter
  r): one beat a frame keeps the wire's pace while a frame lasts 32 core
  cycles or more (BAUD 1 and slower on the SPI); at 16 (BAUD 0, 24 MHz)
  a frame takes 19.1 cycles with the source in the SRAM, the core
  spinning or asleep, and up to 19.7 with the source in the flash: the
  beat's own latency after its trigger, about sixteen cycles
  ([spi.md](spi.md), Bench findings).

## Not covered yet

Driver gaps:
- **Lending a channel at run time** - a transport giving a held channel
  to another while it stays up (a duplex link lending its channels to a
  display while it draws) - is declined: a channel has one owner for the
  whole image ([../design/dma.md](../design/dma.md), "The ownership
  rule"), and a lent channel is a second owner. Measured with a scratch
  protocol on this die: 52,943 hand-overs at the block's completion edge (the receive
  channel at its completion, the transmit channel at its block's end) and
  26,000 mid-block disables with the transmit channel idle, 11.3 M
  borrower requests on the lent channels at 115200, 1 Mbaud and 3 Mbaud,
  636 MB on the loop: no fault (below 5.7e-5 a hand-over, 2.0e-4 an idle
  disable, at 95 %). A receive hand-over at the edge waits for its block
  to fill, which on an idle line is for ever, so the owner then takes the
  other safe moment (the transmit channel idle, the receive channel
  disabled mid-block with its count exact); a transmit hand-over waits
  for its block to drain (10 ms for 1 KB at 1 Mbaud, 88 ms at 115200);
  the masked windows were 213..219 cycles (receive), 311..313 (transmit
  release) and 7 (transmit return). While lent, the Uart receives on its
  interrupt beside its own transmit engine's stream and the borrower's
  requests and handlers: clean at 1 Mbaud when no other handler plus mask
  exceeds two frames, not sustainable at 3 Mbaud, where the receive
  handler takes 110..116 of every 160 cycles and leaves the rest of the
  program too little (the interrupt receiver alone keeps 3 Mbaud,
  sercom.md).
- **Every use outside the two engines' shapes** - declined: a Uart
  receive engine without its transmit engine, an SPI receive engine
  without its transmit engine (the refusals above; a Uart receive channel
  alone beside another owner measured no descriptor fault in 30 minutes,
  and is not offered so that no owner disables a running receive channel
  beside another's), an I2C read phase on the receive engine (not
  measured beside another channel, so not offered: a read runs on the I2C
  host's interrupt), a converter or timer stream (and so no `util/block_stream.hpp`
  realization on this family), a memory-to-memory copy, an
  event-triggered channel.
- **The CRC engine and linked descriptors** - declined with the rest; the
  CRC's data-port erratum 1.10.1 is revision B only.
- **A receive priority knob** (`UartOptions::rx_priority` of
  [../design/dma.md](../design/dma.md)) - not built: the Uart's receive
  engine comes only with its transmit engine and the pair holds the whole
  budget, so no channel exists that a program could rank above the ring.
- **An engine across a standby**: a channel runs with CHCTRLA.RUNSTDBY =
  0, and 25.6.7 makes it software's to suspend such a channel (its
  SERCOM's SYNCBUSY checked) before entering standby and to resume it
  after; nothing suspends it - sentence 2 forbids a suspend beside a
  running channel - and no sleep vote waits for a block in flight. Born
  with the first program that sleeps to standby with an engined
  transport ([platform.md](platform.md)).

Implemented but not bench-verified:
- The rule on the other affected revisions (E and H) and on any die but
  one ATSAMC21J18A at revision F: every bound above is that die's;
  another die of the part, and a part at revision E or H, would measure
  it.
