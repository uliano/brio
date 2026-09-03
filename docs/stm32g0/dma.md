# DMA + DMAMUX (STM32G0)

> **PROVISIONAL.** The controller's whole channel surface, the request
> multiplexer with its synchronization block, its event generation and
> its request generator, the two byte-transport engines and the two
> block-stream engines are implemented and bench-verified. What is still
> missing is in "Not covered yet".

Documents of record: RM0444 Rev 6 - the DMA ch. 10, the DMAMUX ch. 11,
the RCC's AHB enable and reset bits 5.4.9 / 5.4.5, the interrupt table
12.3 (table 61); errata ES0548 Rev 3 items 2.4.1 and 2.5.1..2.5.4, read
on the bench chip's revision Z column, where all five apply. Driver:
`stm32g0/dma.hpp`; the per-part instance, channel and vector facts come
from `stm32g0/device_tables.hpp`, and the REQUEST IDS from the
peripherals that publish them (`Usart<n>::dma_tx_request()`,
`Tim<n>::dma_update_request()` and their kin). Bench suite:
`test_stm32_dma` (12 letters in `z`, 64 verdicts, wireless; letter `u`
outside `z` needs `tools/uart_stress.py`). Family fixture
`test/family_stm32g0/dma.cpp` plus seven negatives under
`tools/check_stm32g0.sh`.

## What the silicon does

**Two controllers, one multiplexer, three vectors.** The G0B1/G0C1 class
has DMA1 with seven channels and DMA2 with five; the G071 class has
DMA1's seven alone, the G031 class DMA1's five. One DMAMUX serves both:
its channels 0..6 are DMA1's channels 1..7 and its 7..11 are DMA2's 1..5
(11.3.2), a hardwired map. Every count above is COUNTED off the device
header's own `DMAn_ChannelK_BASE` / `DMAMUX1_ChannelK_BASE` symbols, not
tabulated. The vectors are table 61's: channel 1 has one to itself,
channels 2 and 3 share one, and ONE line carries DMA1's channels 4..7,
every DMA2 channel and the DMAMUX overrun - and the third line's NAME
differs on all three headers, which is why it is read off the device
select macro in the reserve beside `usart_irq()` and `tim_irq()`.

**A channel is four registers and an enable discipline.** CCR carries
the direction, the circular and memory-to-memory bits, the two
increments, the two access widths, the priority and the three interrupt
enables; CNDTR the number of data items; CPAR and CMAR the two ends.
Everything in CCR but the interrupt enables is READ-ONLY while EN is set
(10.6.3), and CNDTR "must not be written when the channel is enabled"
(10.6.4) - so `configure()` and `set_count()` REFUSE rather than store
into a register the silicon ignores.

**Suspend and resume is not a thing.** 10.4.5 is explicit: disabling an
active channel and re-enabling it without reprogramming "is not
supported by the DMA hardware". The supported sequence is
abort-and-restart - disable, reconfigure, enable, in SEPARATE writes to
CCR - and `load()` is exactly that. What this driver never offers is the
unsupported one.

**A transfer error disables the channel in hardware** (10.4.7) and EN
cannot be set again until TEIFx is cleared. That is the one place a
`false` from this driver means the SILICON refused rather than the
driver did.

**DIR does not say "peripheral" or "memory": it says which side is the
SOURCE.** With DIR = 0 the CPAR side is read and the CMAR side written;
with DIR = 1 it is the other way round, and in memory-to-memory mode the
register names mean nothing at all. `DmaDirection`'s enumerators are
therefore spelled from the source.

**The request vocabulary is not in any header of this pack.** Table 55
numbers 77 request lines by peripheral, and the `DMAMUX_REQ_*` spellings
live in ST's HAL/LL, which this project does not vendor. So the samc
EVSYS ruling applies unchanged: `dma.hpp` owns the FABRIC and takes a
plain request id, and each peripheral publishes its own codes. The
family fixture is where the two publishing drivers are held to one table.

**A DMAMUX channel can do three more things than route.** With SE it
holds the peripheral's request back until an edge arrives on one of 24
synchronization inputs, then lets NBREQ + 1 of them through; with EGE it
emits a one-cycle event every NBREQ + 1 served requests, and table 56
offers those four events BACK as trigger inputs 16..19; and four
separate request-generator channels turn an edge on one of the same 24
trigger inputs into GNBREQ + 1 DMA requests of their own. Trigger inputs
0..15 ARE the EXTI's lines 0..15.

## Errata

All five DMA/DMAMUX items of ES0548 Rev 3 apply to revision Z.

| item | what | how it is answered here |
|---|---|---|
| 2.4.1 | a CGIFx write coinciding with a transfer error loses BOTH the TEIFx flag and the automatic channel disable | **structural**: `DmaFlag::all` does not contain the global bit and `clear()` masks it out, so no caller can write CGIFx. Measured beside it: clearing the three specific flags clears GIFx anyway (10.6.2), so the workaround costs nothing |
| 2.5.4 | a CxCR write that turns synchronization on while SPOL already holds an edge routes the PREVIOUS request id | **structural**: `request()` writes SPOL = 00 with SE = 0, and `request_synchronized()` writes SE and a non-zero SPOL in ONE word. There is no verb that can leave the illegal combination behind |
| 2.5.1 | a SOFx raised by another channel during a CFR write is lost. No workaround | stated on `DmaMux::clear_overrun()`: do not run two synchronized channels that can both overrun |
| 2.5.3 | the same on the request-generator side, during an RGCFR write. No workaround | stated on `DmaMuxGenerator::clear_overrun()` |
| 2.5.2 | with GNBREQ > 1, a trigger landing on the very last request of a batch loses OFx and makes the NEXT batch one single request | a stated obligation on `configure()`: only the application knows its trigger period. The bench's own overrun leg is staged with the channel DISABLED, so it does not sit in that window |

## Types and verbs

`Dma<n>` is the block (bus clock, reset, the two flag registers, the
channel count, `irq(ch)`). `DmaChannel<n, ch>` is one channel, numbered
as the silicon numbers it - from ONE. `DmaMux` addresses the multiplexer
by DMAMUX channel (`DmaChannel<n, ch>::mux_channel` is the map) and
`DmaMuxGenerator<x>` is one of the four request generators.

Four engines sit on top, each owning one channel and knowing nothing
about the peripheral it serves - the data address and the request id are
handed in at `arm()`:

- `DmaTxEngine<n, ch, Elem>` / `DmaRxEngine<n, ch, Elem>` - the byte
  transport pair. They are the two OPTIONAL slots
  `Uart<n, pins, rx, tx, TxEngine, RxEngine>` takes.
- `DmaLoopEngine<n, ch, Elem>` - `util/block_stream.hpp`'s
  **BlockPlayer**: one caller-owned table played into a peripheral for
  ever, on a CIRCULAR channel.
- `DmaPingPongEngine<n, ch, Elem>` - `util/block_stream.hpp`'s
  **BlockSource**: fill one caller-owned buffer while the caller drains
  the other, on a NON-circular one.

`Elem` IS the access width: one `sizeof` decides PSIZE, MSIZE and the
address arithmetic together, so they cannot disagree, and a type that is
not 1, 2 or 4 bytes wide is a compile error at the engine that named it.

### Reading a channel's progress costs nothing

CNDTR is a live register the controller decrements and software may read
at any time (10.6.4). The SAM C21's driver had to SUSPEND a channel,
read a write-back and validate it against an erratum that could corrupt
it; there is no harvest ceremony on this silicon, and `DmaRxEngine::take()`
is one register read and a subtraction.

### One example per use

Memory to memory - no peripheral, no request, runs on the enable:

```cpp
using Copy = brio::DmaChannel<1, 1>;
brio::Dma<1>::bus_clock(true);
Copy::load({.peripheral = src, .memory = dst, .count = 512,
            .config = {.memory_to_memory = true,
                       .peripheral_increment = true,
                       .peripheral_width = brio::DmaWidth::word,
                       .memory_width = brio::DmaWidth::word}});
while (!Copy::flag(brio::DmaFlag::complete)) {}
Copy::stop();
```

A waveform played for ever, with the CPU out of the path:

```cpp
using Player = brio::DmaLoopEngine<1, 1, uint32_t>;
Player::arm(Tim2::ccr_address(0), Tim2::dma_update_request());
Tim2::interrupts(Tim2::update_dma, true);
Player::start(duty_table, 8);          // and that is all: CIRC does the rest
extern "C" void DMA1_Channel1_IRQHandler() {
    if ((Chan::isr() & brio::DmaFlag::complete) != 0u) { Player::complete(); }
}
```

A sampled stream into a `BlockRelay`:

```cpp
using Source = brio::DmaPingPongEngine<1, 4, uint32_t>;
using Relay  = brio::BlockRelay<brio::Stm32Platform, Subs, Source>;
Source::arm(&TIM2->CNT, Tim3::dma_update_request());
Source::start(buffer_a, buffer_b, 32);
// in the channel's handler, after Source::complete():
brio::post<Relay>(brio::BlockDone{});
```

A USART that moves its bytes without the CPU - the slots are empty by
default and cost an engineless image nothing:

```cpp
using Serial = brio::Uart<2, pins, 64, 256,
                          brio::DmaTxEngine<1, 6>, brio::DmaRxEngine<1, 7>>;
extern "C" void DMA1_Ch4_7_DMA2_Ch1_5_DMAMUX1_OVR_IRQHandler() {
    (void)Serial::dma_isr();
}
// ... and somewhere in the loop, or on a TimeEvent:
(void)Serial::harvest();
```

## The block stream's second implementation

`util/block_stream.hpp` was written against the SAM C21 and BEFORE this
implementation, on purpose, so that friction would show up as "this
concept does not fit" instead of as silent divergence
(`docs/design/block-stream.md`). Both concepts are satisfied here
UNCHANGED - not one line of `util/` moved - and the interesting part is
what the measurement said on the way.

**`BlockPlayer` fits circular mode exactly, and gains by it.** The SAM's
controller has no circular mode, so its loop engine re-armed from a
completion interrupt: one interrupt per lap, and a window at every lap
boundary in which the peripheral was unserved. Here CCR.CIRC reloads
CNDTR and both current address registers in hardware, so the interrupt at
the wrap only COUNTS the lap. Measured: a circular channel with its
interrupt DISARMED and its NVIC line masked was still enabled 25 ms
later with CNDTR moving, and the captured block was self-checking to the
tick. The sentence in the design doc that says a player "publishes
nothing per lap and `laps()` moving is the one fact that says the stream
is alive" is exactly right here - and cheaper than it was there.

**`BlockSource` does NOT fit circular mode, and the reason is the
doctrine rather than the API.** Circular plus the half-transfer flag
looks like a ping-pong for free: one buffer, two halves, HT and TC as the
two edges. It cannot honour the contract's central trade - "an overrun
means the source SKIPPED a block rather than tear one" - because A
CIRCULAR CHANNEL NEVER STOPS. When the caller still holds the half the
controller is about to write, the only moment software could intervene is
the interrupt AFTER the edge, and by then the controller has already
begun writing that half. The tear happens while the software is deciding
not to allow it, and it is invisible: nothing in the block says which of
its elements belong to this lap and which to the next.

Measured, with a handler whose whole body is "disable the channel and
read CNDTR" at the half-transfer edge:

| request rate | elements already written into the next half when the handler stopped the channel |
|---|---|
| 10 kHz | 0 |
| 2 MHz | 6 |

So under CIRC the guarantee would be a hope about interrupt latency, and
its failure would be silent. `DmaPingPongEngine` therefore uses a
NON-CIRCULAR channel that stops itself at the end of every block: the
swap into the other buffer happens with the channel idle, and the
guarantee is structural. The suite reads CCR back while the engine
streams and finds CIRC clear, so the claim is the silicon's and not the
comment's.

**What the non-circular choice costs, and why it is smaller here.**
Between a completion and the handler's next `start()` the peripheral is
unserved. On this controller 10.4.3's request is a LEVEL the peripheral
holds until it is acknowledged, so a request raised inside that window is
served LATE rather than lost; what can still be lost is a SECOND arrival
inside one window, and that loss is the peripheral's own overrun flag to
report - which is exactly where the contract already puts it.

**No sentence of `docs/design/block-stream.md` had to change.** The one
that this implementation makes sharper is the concepts' own framing:
they are about BLOCKS and not about DMA, and the proof is that the
same two concepts are met by a controller whose natural streaming mode is
the one the contract cannot use.

## Bench findings

Everything below is `test_stm32_dma` on a Nucleo-G0B1RE at 64 MHz, no
wires.

**A REQUEST IS A LEVEL SERVED ON ENABLE, NOT AN EDGE LATCHED ON THE
RISE** - the opposite of the SAM C21's DMAC, and the reason this driver
has no `kick()`. Staged on USART1 brought up with its pads never claimed
(so nothing leaves the die) and TXE therefore standing: a channel armed
over that standing request moved its whole four-byte block with no
software trigger of any kind, and the control - the same channel over the
same standing TXE with CR3.DMAT clear - moved nothing at all. The SAM
campaign's wedge (a channel enabled, its peripheral asking, and not one
beat moving) has no analogue here.

**Two memory-to-memory channels ALTERNATE, and the software priority
does not enter into it.** 10.4.4 says so in a clause easy to read past:
re-arbitration between every single transfer, and "the DMA arbiter
automatically alternates and grants the other highest-priority requested
channel, WHICH MAY BE OF LOWER PRIORITY than the memory-to-memory
channel". Measured over 512-word blocks: at the first finish the loser
was within 3 of the winner in ALL THREE arrangements - very-high against
low, low against very-high, and equal. What is left is the hardware
tie-break, and it is exact: channel 1 was ahead every time, INCLUDING
when it was the low-priority channel. The lower index wins and no
register configures it.

**THE PL FIELD'S OWN EFFECT IS DECLINED, NOT DISPROVED.** Two channels
over-requested at 32 million update events a second EACH - five times the
throughput measured below - still finished within one or two counts of
each other whichever way the two levels were set. This bench has
therefore measured the ALTERNATION and the INDEX and has NOT shown PL
reordering anything; a stimulus that keeps one channel's request standing
while the other's is down is what would show it, and none of this suite's
sources does that.

**Throughput.** 512 words (2048 bytes) memory to memory in 2638 cycles =
5.15 cycles a word, about 49.7 MB/s at 64 MHz - which is the two AHB
accesses 10.4.3 describes plus arbitration, and about four times the
core's own copy loop.

**Table 51's alignment rules hold as printed.** A byte source into a
halfword destination is ZERO-EXTENDED, one item per address step of two
(0xC0 reads back 0x00C0); a word source into a byte destination keeps the
LOW byte of each word (0x11110003 reads back 0x03) - a truncation, not an
error. PINC clear turns a block into a FILL.

**A transfer error is exactly what 10.4.7 promises.** A read of
0x30000000 - between the SRAM and the peripherals, belonging to nobody -
raised TEIF and GIF together and the channel came back DISABLED with
nothing in software having noticed first. EN could not be set again
until TEIF was cleared, and could immediately after.

**The three shared vectors confine themselves.** Channel 3's four flag
bits were clear throughout a block channel 2 ran on the SAME vector, so
an ISR body that reads only its own group cannot consume its
vector-mate's completion. And an UNARMED flag is set by the hardware
anyway: the ISR body reports and clears only what is armed, so a poller
still finds a completion nobody asked to be interrupted for.

**The request generator turns an edge into requests, on a trigger with no
pad.** Table 56's input 22 is TIM14_OC, and TIM14 needs no pin to produce
one: ten triggers at 1 kHz with GNBREQ + 1 = 4 moved exactly 40 words,
never a stray one. With the channel left DISABLED so no request is ever
served, the second trigger set OFx as 11.4.5 warns, and RGCFR cleared it.

**A SOFTWARE EXTI TRIGGER DID NOT REACH THE REQUEST GENERATOR - recorded,
not explained.** Table 56 makes trigger inputs 0..15 the EXTI's lines
0..15, and this family's EXTI has SWIER, so a request generator ought to
be reachable from software with no pad at all. Five SWIER pulses on line
9, with the line's rising trigger selected and BOTH its interrupt and its
event mask set, moved ZERO words on a channel that the TIM14_OC trigger
drove perfectly in the same letter. The data verdict is DECLINED with
the finding printed. What this bench has not separated: whether the
DMAMUX watches a signal SWIER does not drive, or whether a pad left in
analog mode (which the EXTI campaign found blinds a line) blinds this
path too.

**EGE makes one channel pace another through the fabric alone.** With a
multiplexer channel set to emit an event every four served requests and a
request generator triggered by `dmamux_evt0`, a paced 64-request stream
on DMA1 channel 1 moved exactly 16 words on channel 5 - which has no
peripheral of its own and never sees an interrupt.

**The console's own transmit engine saturates the wire, and at 115200 the
per-byte feed costs nothing.** A kilobyte took 88.97 ms fed byte by byte
(11510 B/s) and 88.98 ms fed in bulk (11508 B/s), against the 11520 B/s
115200 8N1 carries. The samc campaign measured the per-byte pump losing a
third of the wire - but it measured it at MEGABAUD; here the wire is five
hundred times slower than the pump, the ring is always full when a block
ends, and every block the engine gets is a long one.

**The timer round trip.** An eight-entry duty table played into TIM2's
CCR1 on its own update request, at a 16 kHz PWM, reads 521..522 per mille
off LD4's own pad against the table's mean of 525 - with the CPU touching
not one compare value; the lap count moves at exactly the predicted
2000 laps a second, and stopping the player leaves the pad holding one of
the table's own entries. In the other direction a ping-pong engine
drained TIM16's capture of the LSI - a real ~32 kHz stream reached
through TISEL with no pad at all - four whole blocks of 32, every
consecutive pair 30 us apart, nothing torn and no overrun.

**`BlockRelay` inside a real kernel.** Ten caller-owned blocks of 32
words travelled from the ping-pong engine to a subscriber as
`Lease::dispatch` loans, every sample inside every block exactly one pace
period (32000 ticks at 2 kHz) after the one before it, every SEAM between
blocks the same, zero engine overruns, and the relay's `published()`
equal to the engine's own `laps()`. Then the other half of the contract:
with nobody draining, the engine STALLED and counted an overrun rather
than write into the block the caller held, and one `release()` brought
the stream back.

### The VCP's ceiling (letter `u`, with `tools/uart_stress.py`)

The board's own USART reaches every rate in the ladder; what the
ST-LINK's virtual COM port carries is another question, and only the host
can answer it.

| baud | board -> host (host verified) | host -> board (board verified) |
|---|---|---|
| 115200 | byte-exact, 7103 bytes | byte-exact, 1792 bytes |
| 460800 | byte-exact, 12000 bytes | byte-exact, 6210 bytes |
| 921600 | byte-exact, 12000 bytes | byte-exact, 11040 bytes |
| 2000000 | **corrupt from byte 6145** | **17963 of 18059 bytes wrong** |
| 3000000 | **7 bytes of 12000 arrived** | the host could not even pump a second chunk |

So this VCP is good to 921600 in BOTH directions and not to 2 Mbaud in
either. The board's own transmitter emitted its full window at every rate
including 3 Mbaud (12000 bytes in 39.66 ms = 302600 B/s, 101 % of nominal
- the arithmetic, not the wire), so the ceiling measured here belongs to
the BRIDGE and not to the USART or its engine. `hw_overruns` and the
frame counter stayed at ZERO throughout, at the good rates and the bad
ones alike, which says the corruption happens upstream of the receiver's
own error detection - the bytes that arrive are well-formed frames
carrying the wrong data.

An operational note on the tool: this letter's ten legs take longer than
`uart_stress.py`'s own 120-second per-letter window, so the script prints
every leg and then reports "no tally". The board is not stuck - it prints
its tally and answers its prompt immediately afterwards; the letter is
outside `z` and the suite's score does not depend on it.

### The synchronization block (letter `k`)

dma.md carried this as built-but-never-run since the DMA campaign, and
what it wanted was a stimulus that was already on the board: **table
57's synchronization input 22 is TIM14_OC**, the same signal table 56
offers the request generator as trigger input 22 and the same signal
letter f already produces with no pad at all. So TIM3's update event is
the REQUEST at 20 kHz and TIM14's OC1REF is the SYNC at 1 kHz.

In a five-millisecond window:

| arrangement | words moved |
|---|---|
| unsynchronized (the control) | 64 - the whole transfer |
| SE, NBREQ 4, rising | 18 |
| SE, NBREQ 4, falling | 20 |
| SE, NBREQ 4, **both** | 39 |
| SE, NBREQ 1, rising | 5 |

Five sync edges times four requests is twenty words where the same
channel unsynchronized saturates. `SPOL` is a real selector - either
single edge serves the same count and BOTH serves twice as many, which
is the only reading of 11.6.1's three codes a square wave can tell
apart - and **NBREQ is the count and not the count minus one**, which is
what `DmaMuxSync::requests` claims and this measures.

**A SYNCHRONIZATION OVERRUN IS REAL AND REPORTED.** With eight requests
asked for per edge and only about half a request arriving between edges,
`SOF` for that multiplexer channel stands and `CFR` clears it; with
`SOIE` set the same arrangement delivered **7 interrupts** on the third
NVIC line. TWO LEGS, and the reason is that line: the flag is read with
the interrupt off and the shared handler's sweep GATED, because a
handler that clears `SOFx` is the thing that would hide it.

**AND THE VECTOR IS NOT THIS LETTER'S TO ARM.** The DMAMUX overrun
shares the third line with DMA1's channels 4..7, which is where this
suite's own console engines live: enabling it is a no-op and disabling
it afterwards takes the console's transmitter down with it. The first
version of the letter did exactly that and the board went silent
mid-letter.

**The request generator's other two polarities**, which this document
also listed: over ten TIM14 periods, rising moved 10 words, falling 10,
both 20.

## Not covered yet

Driver gaps:

- **Peripheral-to-peripheral transfers** in the sense of 10.4.5's first
  bullet (a peripheral's request pacing a transfer between two OTHER
  registers) are reachable through the existing verbs and exercised by
  this suite's paced letters, but no vocabulary names the arrangement.
- **`Dma<n>::reset()` resets the DMAMUX with the controller** (the RCC
  bit's own description), so a program with two live controllers cannot
  use it. Nothing here refuses the call; the caveat is stated on the verb
  and the suite never takes it.
- **No sleep story.** Whether a channel keeps running in Sleep or Stop,
  and what `RCC_AHBSMENR.DMAxSMEN` really buys, waits for a PWR driver -
  there is none in this stratum yet.
- **DMA2's channels are compile-checked and only channel 1 is bench-run.**
- **No linked-list or repeated-block vocabulary**, because this
  controller has none: a channel is one block, and the only repetition it
  offers is CIRC.
- **A one-shot burst vocabulary** in `util/` - a finite capture started
  and stopped - is still absent, and this campaign found no reason to
  change that (`docs/design/block-stream.md` says why).

Implemented but not bench-verified:

- the widths on DMA2, and every channel of DMA1 above 5 except as the
  console's own two engines.

Errata not staged, and why: 2.4.1 is a same-cycle coincidence between a
hardware error and a CGIFx write, and the write does not exist in this
driver; 2.5.1 and 2.5.3 need two channels that can both overrun at once,
which is the configuration their own workaround says to avoid; 2.5.2
needs a trigger landing on the last request of a batch above two, which
this suite's overrun leg deliberately steps around.
