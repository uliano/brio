# SERCOM - USART mode (SAM C21)

Documents of record: SAM C20/C21 data sheet DS60001479M - SERCOM
common ch. 30 (the baud generator, 30.6.2.3 table 30-2), USART
ch. 31 - and errata DS80000740S items 1.17.15 and 1.17.16, both
encoded in code (1.17.4 and 1.17.14 are named where their fields
live). Driver: `samc21/sercom.hpp` (`Sercom<n>` resource +
`Uart<n, pads, rx, tx, TxEngine, RxEngine>` task, the engine slots
optional and ONE of them at most). The family fixture is `test/family_samc21/sercom.cpp` plus
its negatives under `brio check samc21`; the bench suite is
`test_samc_uart`, driven from the host by `brio stress`.

## What the silicon does

**One interrupt vector for the whole instance.** DRE, TXC, RXC, RXS,
CTSIC, RXBRK and ERROR all share the instance's single NVIC line,
where the AVR DA/DB's USART has two vectors and two ISR bodies. And
DRE is a CONDITION, not an event: it reads 1 whenever the transmit
buffer is empty, which is most of the time, so a handler acting on raw
INTFLAG would run the transmit path on every receive interrupt. The
mask that matters is INTFLAG AND INTENSET.

**Instance count is the family's one package difference.** The E
package bonds four SERCOMs (0..3), the G and J six - read from the
device header, refused beyond. The GCLK core-clock channel is a
per-instance header constant, not a formula: SERCOM0..4 sit at
19..23, SERCOM5 at 25 (24 is its private slow channel where the
others share 18).

**TxD lives on PAD[0] or PAD[2], and nowhere else** (CTRLA.TXPO);
RxD may be any pad. A two-wire link therefore cannot swap its pads
when a board turns out crossed - "the other way round" is a different
pad pair, not a configuration. The device header adds a naming trap:
its enumerator `TXPO_PAD1_Val` names the CODE 0x1, and that code
routes TxD to PAD[2].

**The frame is LSB-first on the wire - but the register's reset is
not.** CTRLA.DORD resets to MSB-first; a standard UART frame is
LSB-first and the chapter's own init sequence says to set the bit.
The measured consequence of getting this wrong: every byte arrives
exactly bit-reversed at the correct baud (0x0D 0x0A 'S' reads back
0xB0 0x50 0xCA), with every other register correct - the driver's
`UartFormat` defaults the bit and a family static_assert keeps it from
regressing.

**Enable-protection and synchronization are both real.** CTRLA,
CTRLB and BAUD accept writes only while the instance is disabled -
an enabled-time write is DISCARDED (31.6.2.1). SWRST, ENABLE and
CTRLB cross into the peripheral clock domain: 31.8.10 promises an
APB error for a CTRLB write while the previous one is in flight, and
enabling the instance CLEARS CTRLB.TXEN/RXEN, raising SYNCBUSY.CTRLB
until each direction is really up - verified verbatim in 31.8.2, and
the reason the driver's enable waits BOTH sync bits. Two errata
complete the picture: SWRST does nothing while ENABLE = 0 (1.17.16 -
so a reset from the disabled state must enable first), and the ERROR
interrupt does not wake the device (1.17.15 - so it is never enabled
at all; errors are taken on the RXC path, where STATUS must be read
before DATA anyway, 31.8.11).

**The baud generator, 16x arithmetic** (table 30-2):
`BAUD = 65536 x (1 - 16 x f_baud / f_ref)`, f_ref being the
instance's GCLK core clock, with f_baud <= f_ref/16. BAUD = 0 is a
LEGAL value - the fastest rate - which is why the driver's arithmetic
returns an optional rather than overloading zero. TXC is exact:
cleared by every DATA write and set only when the shifter empties with
nothing queued - "the last byte is on the wire", no timing guesswork.

**A SERCOM input pin can only pull DOWN** (31.5.1, verified
verbatim): PULLEN still works under the peripheral function, but the
pull-up is not available - an idle RxD line must be held high by
whatever drives it.

## Types and verbs

- **`Sercom<n>`** - the resource: bus clock (MCLK mask) and core
  clock (GCLK channel) wiring, reset/enable with their bounded sync
  waits, `configure(SercomUartConfig)` writing the whole
  configuration while disabled, the flag verbs with the W1C half
  spelled out (RXC and DRE are conditions and ignore writes),
  `pending()` = INTFLAG AND INTENSET (the shared vector's one
  question), STATUS with its read-before-DATA discipline, DATA,
  `flush_rx()`.
- **`UartPads`** - which pad carries each direction plus where those
  pads come out (`SercomPadPin`: port, pin, PMUX function). The pad
  side is checked exactly (`uart_pads_valid` refuses TxD anywhere but
  PAD[0]/PAD[2] at compile time); the pin side as far as this header
  can know it - that a given PIN reaches that PAD is the package's
  I/O table, and an application can static_assert its claim against
  the device header's own `MUX_...` constants.
- **`UartFormat`** - bits (5..9), parity (one enum, because FORM and
  PMODE must agree), stop bits, and the LSB-first default above.
- **`Uart<n, pads, rx_size, tx_size, TxEngine, RxEngine>`** - the
  task: two SPSC rings (lock-free at any size here - `atomic_width` 4
  - so 64/256 are console-class defaults, not a ceiling),
  `init(clock, baud, format)` speaking hertz off the clock tag,
  `isr()` as the ONE handler body honouring the edge-return contract
  (true on the RX ring's empty-to-non-empty transition - the kernel
  wakeup), try-semantics `write_byte` and `write_bulk` (a run: as
  many as fit, its first byte pushed and DRE armed before the rest is
  copied and armed again behind it, or with an engine the run queued
  whole and the engine pumped once - the verb `print` hands every
  string and number), `read_byte`, `read_bulk` and `read_span`/`consume` (the
  receive run in place), error counters,
  `rebase(hz)` for the day a dynamic clock exists, `set_baud(hz,
  baud)` (a new rate under the running port, once TX is idle),
  `release()`. Init
  order is deliberate: clocks, reset, configure,
  enable, and only THEN the pads to the SERCOM - the transmitter
  idles high before the pad leaves PORT, so no glitch start bit
  reaches the wire. `ByteTransport` and `ClockUser` are
  static_asserted. The two engine slots default to `NoDmaEngine` -
  see "The optional DMA engines" below.
- **The resource's DMA constants** - `Sercom<n>::dma_rx_trigger()` /
  `dma_tx_trigger()` (table 25-2's codes, read off the device header
  per instance beside the GCLK id and APB mask - the family fixture
  static_asserts them against `samc21/dmac.hpp`'s own spelling so the
  two cannot drift) and `data_address()`, the one register a transfer
  ever touches.

## The optional DMA engines

An application that wants DMA on a direction includes `samc21/dmac.hpp`
alongside this header and names an engine with its channel - on ONE
direction, the other keeping its interrupt:

```cpp
using Serial = brio::Uart<5, console_pads, 64, 256, brio::DmaTxEngine<0>>;
using Logger = brio::Uart<5, console_pads, 256, 64,
                          brio::NoDmaEngine, brio::DmaRxEngine<1>>;
```

The engines are POLICIES, not features of the task:

- **Zero when absent.** `NoDmaEngine` (the default) is a tag with
  `present = false`; every engine branch sits behind `if constexpr`,
  so a Uart that names no engine carries no test, no state and no
  code - an engine-less image is byte for byte what it would be with
  no engine slots at all. sercom.hpp never includes dmac.hpp (the
  engines live THERE), so a program with a serial port does not carry
  descriptor tables.
- **The trigger replaces the interrupt.** DRE for the transmitter and
  RXC for the receiver are the SAME condition as the DMA trigger, so
  the direction that has the engine does not arm its interrupt -
  both armed would serve every byte twice.
- **ONE ENGINE AT MOST: erratum 1.10.4 as a compile error.**
  DS80000740S 1.10.4, live on this silicon: "When using concurrent
  channels triggers, the DMAC write-back descriptors may get
  corrupted", and its one workaround: "Multiple transfers must only be
  sequenced using linked descriptors on a single channel." The erratum
  names no other remedy - no placement of the write-back section, no
  priority, no arbitration scheme - and a port's two engines are two
  channels on two triggers clocked by the two ends of the wire, which
  nothing sequences onto one channel. So the pair is refused at the
  template argument (`uart_engines_not_concurrent()`, a static_assert
  with the erratum as its message; the family fixture's negative
  `uart_both_engines` holds it). A runtime rule - the second engine
  refused while the first's channel runs - would name an engine it
  could never use: the receive channel waits enabled for as long as
  the port is up. What the pair cost, measured in the Bench findings
  below, is why it is refused rather than counted: the erratum struck
  in nineteen echoes of twenty, and wrote this SERCOM's registers. A
  duplex link that wants bulk both ways takes the transmit engine and
  keeps the receiver on RXC, which a run-at-a-time consumer holds to
  2 Mbaud (letter h). Channels of OTHER owners triggered beside the
  engine are the program's to sequence (dmac.md).
- **TX drains the ring in blocks.** `write_byte`, `write_bulk` and
  `print` are unchanged; the engine is handed the ring's contiguous run
  (`read_span`, design/ring.md) and its completion interrupt consumes
  exactly the block it carried and starts the next - a wrapped ring
  goes out in two blocks. The app's DMAC handler routes completions
  with `dma_isr(channel)`, which answers false for channels that are
  not this transport's. THE MASK COVERS THE CLAIM: a block start (from
  `write_byte`/`write_bulk` in main context or from the completion in
  the handler) takes the engine's reservation - a test-and-set of its
  busy flag, nine instructions under PRIMASK - and reads the run and
  programs the channel unmasked, because no completion of an engine
  that has no block in flight can land under it. The block itself is
  three stores into the descriptor slot and the channel's enable
  (dmac.md); an engined direction's ring is at most 65535 bytes, a run
  being one block and BTCNT sixteen bits (refused at compile time).
- **RX fills the ring's free run and is HARVESTED on the caller's
  clock.** A receive block completes only when the buffer fills -
  on an idle line, never - so arrival is not an event anyone is told
  about: `harvest()` suspends the channel, reads the validated
  write-back (erratum 1.10.4, see dmac.md), publishes the fresh
  bytes, and returns the same empty-to-non-empty edge `isr()` has, so
  the same kernel glue works. Pacing is WHOEVER OWNS THE PORT's
  policy - a kernel TimeEvent every few ticks is the shape brio
  expects - and each harvest costs ~15 us of masked interrupts, 44 us
  when the suspend never lands (dmac.md).
  `harvest()` hands the channel a new run whenever the SILICON says it
  is not running one, not only when the engine's own beat count says
  the buffer filled: a reading that was refused leaves that count
  behind, and a re-arm rule that trusted it alone would never fire
  again.
- **THE STANDING REQUEST, AND WHY NEITHER DIRECTION KICKS.** A
  peripheral asserts its DMA request as a LEVEL - "my transmit buffer is
  free", "I have a character" - and the DMAC turns that level into a
  pending trigger when it RISES. A block armed while the level is
  already high could, on that reading alone, wait for an edge that has
  gone by. It does not: a rise while the channel is DISABLED with its
  trigger selected is latched and served on the next enable, and so is
  the claim's selection of the trigger onto a request already standing
  (dmac.md), and an engine keeps its trigger selected from `arm()` on.
  So the enable fires the first beat by itself, and a software kick on
  top of it is lost when that trigger is still pending and a SECOND beat
  when its beat has started (PEND clears as a beat starts, 25.8.23) -
  measured both ways (Bench findings): a transmit kick doubled a byte
  into a full DATA, and a receive kick read DATA twice, the stream
  arriving LONGER than it was sent. Neither `pump_tx()` nor the receive
  re-arm kicks; every wedge measured on this transport carried a
  corrupted write-back (erratum 1.10.4), which the DEAD-BLOCK PREDICATE
  answers on the refused-byte path below: a channel that waits ENABLED
  with no trigger pending and no beat moving while DRE - its trigger -
  and TXC both stand cannot be running, so the block is abandoned
  (dmac.md) and counted in `dma_faults()`. The channel is asked and not
  the engine's own "in flight", which stays true from a block's last
  beat until its completion is heard and is true again for a block the
  handler has just started: with main context preempted for a character
  time across a completion, that test abandoned live blocks at 2 Mbaud
  with ONE channel in the image (Bench findings). The three readings are
  taken under the mask. The channel that corrupts a block is another
  owner's - the transport's own pair is refused - and a corruption that
  leaves DRE clear (DATA full, or the transmitter switched off by a
  stray write) is beyond the predicate.
- **A REFUSED BYTE STILL NUDGES.** `write_byte()` returning false is
  the state in which nothing is draining the ring, and `print()`
  answers that false by trying again for ever - so the refusal path
  arms DRE (or pumps the engine) instead of returning silently, and
  `write_bulk()` does the same when nothing fitted. An EMPTY run
  through `write_bulk()` is a legal call for exactly this: it queues
  nothing and takes the same path, which is how a loop waiting for the
  ring to drain still gives the transport its push.
- **What is traded away, and cannot be given back:** per-byte error
  attribution. With RXC armed, STATUS is read before each DATA and a
  corrupted byte is dropped precisely; with the channel consuming
  RXC, STATUS is read once per harvest and its errors are counted
  against the harvested run, not a byte. A protocol with its own
  framing does not care; a console that wants exact frame-error
  attribution should not take an RX engine.

`brio::Dmac::init()` comes before the engined `init()`: the engines
configure their channels into a block that must already own its
descriptor tables.

## How to use it

```cpp
constexpr brio::UartPads console_pads{
    .tx = brio::SercomPad::pad0,          // PB30 - the silicon's choice
    .rx = brio::SercomPad::pad1,          // PB31
    .tx_pin = {'B', 30, brio::PinFunction::d},
    .rx_pin = {'B', 31, brio::PinFunction::d},
};
using Serial = brio::Uart<5, console_pads>;
constexpr Serial serial;

extern "C" void SERCOM5_Handler() {
    if (Serial::isr()) { brio::post<SerialLines>(brio::RxActivity{}); }
}

int main() {
    SysClock::init();
    Serial::init(clock, 115200);
    brio::enable_interrupts();
    brio::print(serial, "hello", brio::crlf);
}
```

## Bench findings

- The arithmetic is byte-exact on the wire: BAUD(48 MHz, 115200) =
  63019, and the hardware's own readback through `actual_baud`
  reports 115219 Hz - exactly what the inverse arithmetic predicts
  from that register value, to the hertz.
- A full duplex round-trip at 115200 over the board's CH340: banner,
  command parsing, replies, timed responses coherent with the
  SysTick timebase; error counters all zero after the exchanges.
- The DORD fact above is measured, not deduced: right baud,
  bit-reversed bytes, every other register verified over SWD in one
  halt-and-dump.
- The enable-clears-TXEN/RXEN clause and the pull-down-only clause are
  both verified against the data sheet text verbatim - neither is
  folklore.
- The engined transport, live on the console port's SERCOM5: print()
  through the TX engine is byte-exact on the wire (a six-line banner
  went out as seven DMA blocks - the ring wrap served as two spans,
  exactly as designed); the RX engine with a tick-paced harvest
  served a typed burst unchanged with the RXC interrupt never armed;
  the transmit engine beside two memory-to-memory channels sprayed for
  three seconds (`test_samc_dma` j: the erratum corrupting the churned
  channels' write-backs, 81 readings refused and 23 suspends lost in
  68409 rounds) carried 694 lines of 694 intact and in order, no block
  of its own abandoned (the full account is in dmac.md).

**THE THREE SHAPES, BYTE FOR BYTE** (suite `test_samc_uart` with
`brio stress` at the other end; the pattern is a 32-bit
xorshift both ends generate, so a lost byte is located and not merely
counted). Interrupt on both directions, or DMA on one of them, as an
echo of 8384 bytes at 115200 in a 1.2 s window, each DMA shape twenty
times:

  | transport      | received    | notes                                    |
  |----------------|-------------|------------------------------------------|
  | irq TX + irq RX| 8384        | byte-exact both ways                     |
  | DMA TX + irq RX| 8384        | byte-exact both ways, 20 of 20, no block abandoned |
  | irq TX + DMA RX| 8381..8384  | byte-exact or a few short: a gap         |

  The interrupt receiver is exact; the DMA receiver can lose a byte at a
  block boundary, and WHERE it loses them is its contract rather than a
  defect - a block that fills has no run to continue into until a
  harvest re-arms the channel, so whatever arrives in that gap is gone.
  It is measured, not hidden: the suite prints the position of the first
  missing byte. It never receives MORE than was sent (it did, while the
  re-arm kicked: below).

  THE FOURTH SHAPE, DMA on both directions, is a compile error (erratum
  1.10.4, "The optional DMA engines" above), and this is what it did
  when it built: the same echo, twenty runs, the erratum in nineteen of
  them. Seventeen passed the suite's verdicts while abandoning one to
  seven transmit blocks each (the dead-block predicate, counted in
  `dma_faults()`), the host receiving up to 138 bytes MORE than it sent;
  two of those had SERCOM5's INTENSET scribbled (0xA8 and 0x38, where the
  engined transport arms nothing); one ended with the transmitter WEDGED
  - its channel enabled, its write-back its own, DATA full and TXC clear,
  so the predicate could not fire - and the window never drained; and
  one left the board silent until it was re-flashed. One run in twenty
  was byte-exact.

- **The duplex link the refusal leaves**, the transmit engine beside the
  interrupt receiver, twenty runs of letter h: at 1 Mbaud 61500 to
  62000 bytes each way, byte-exact on the board's side every time, no
  hardware overrun; at 2 Mbaud 87000 to 88000, the interrupt receiver
  overrunning in hardware in ten runs of twenty (one to three bytes,
  each counted in `hw_overruns`) - one RXC entry per 240 cycles and two
  characters of FIFO is its edge - and no transmit block abandoned in
  any run of either rate. The host's own view of the echo above 115200 is the
  bridge's and is not judged: the plain transport's echo at 1 Mbaud,
  byte-exact on the board, reached the host 635 bytes short.

- **Rates, through the interrupt transport, echoing:** 115200 and
  1 Mbaud are byte-exact; 3 Mbaud loses, and the loss is ACCOUNTED FOR
  in `hw_overruns` rather than silent - one RXC interrupt per byte is
  300000 a second, which the two-deep FIFO does not survive.
- **Frame formats, against a host that can speak them:** 8E1, 8O1, 8N2,
  7E1 and 7N2 all carried the stream byte-exact both directions with no
  receive error raised. Frames narrower than eight bits carry only their
  low bits, and both ends mask accordingly.
- **The error paths, provoked and counted.** A host at 57600 into a
  115200 receiver raises framing errors (47 in a 1135-character window)
  and the stream comes back byte-exact at the right rate afterwards.
  THE FRAME MISMATCH IS ASYMMETRIC, which the chapter does not say: a
  host at 8E1 into an 8N1 receiver raises framing errors freely (198 in
  1210 characters - the extra parity bit lands where the receiver's stop
  bit belongs), while the reverse - a host at 8N1 into an 8E1 receiver -
  raises NOTHING AT ALL: 2432 characters arrived with frame and parity
  counters both zero. The receiver reads the sender's stop bit as the
  parity bit and finds the idle line where its own stop bit belongs, and
  nothing about that is illegal. A UART cannot be trusted to notice that
  its peer is missing a bit; it notices an extra one.

**How fast the link really goes, and what it costs**, throughput
reported by the board while the host checks every byte. The
measurements are of the BENCH LINK - an ADuM1201 isolator and a CH340
bridge between the pads and the PC - as much as of the driver.

- **3 Mbaud works**, which is the generator's own ceiling at 48 MHz
  (16x oversampling, BAUD 0). A raw polled transmit - no ring, no
  interrupt, no DMA - moved 64 KB byte-exact at 299251 B/s, 99.75% of
  nominal. So neither the isolator nor the bridge is the limit.
- **2.5 Mbaud is a hole, not a ceiling**: it fails while both 2 M and
  3 M are byte-exact. The bridge's divisor arithmetic has no exact
  2.5 M and the nearest is some 4% off, outside what a UART tolerates.
  A failure at one rate says nothing about the rate above it.
- **The per-byte API is what limits a fast link.** A loop over
  `write_byte()` pays a transport nudge every byte - arming DRE, or
  `pump_tx()` with an engine. That plateaus at 98.4 kB/s (about 1 Mbaud
  equivalent) at EVERY rate from 1 Mbaud up, the wire idling while the
  CPU catches up. Fed this way the DMA transmit engine is SLOWER than
  the interrupt, 57-64 kB/s at 92% CPU, because a pump_tx() per byte
  starts a block for one byte.
- **`write_bulk()` is what makes the transmit engine worth having.** The
  same 64 KB at 3 Mbaud: 169343 B/s at 75% CPU through the interrupt
  transport, and 297890 B/s - 99.3% of the wire - at 9% CPU through the
  engine. At 1 Mbaud the engine saturates the wire at 5% CPU against
  70% for the per-byte interrupt path.
- **With the engine, transmit is limited by the BAUD GENERATOR and
  nothing else.** Measured across four rates, the engined bulk path
  costs 4% of the CPU at 46 kB/s, 5% at 100 kB/s and 8% at 298 kB/s -
  a straight line whose slope is **7.6 CPU cycles per byte**, about
  1.6% per 100 kB/s, over a fixed ~3% that belongs to the measuring
  loop rather than the transport. Extrapolated, the CPU would not
  saturate until roughly 6 MB/s, some twenty times what this peripheral
  can emit at all. And those 7.6 cycles are the COPY INTO THE RING, not
  the DMA: a path that handed the engine the application's own buffer
  would not pay them either.
- **Round trip, and it has a different ceiling from transmit.** Echoing
  through the interrupt transport is lossless to 1 Mbaud when the ring
  is drained a byte at a time, and to **2 Mbaud** when `read_bulk()`
  drains it - both directions at once, zero loss. The two ceilings fail
  differently, and the difference names the cause: the per-byte consumer
  loses in the SOFTWARE ring (`rx_overruns` climbs, `hw_overruns` stays
  0), while at 3 Mbaud the bulk consumer loses in the HARDWARE
  (`hw_overruns` 143, `rx_overruns` 0). Bulk fixes the consumer; what
  breaks at 3 Mbaud is the FILLER, which is still one RXC interrupt per
  byte - 300000 a second, more than the two-deep FIFO survives. CPU
  during the echo sits at 55-61% at every rate from 1 to 3 Mbaud, so it
  is the interrupt RATE that gives way and not the total work.
  (The ~50-58 kB/s each way these runs report is the HOST's USB
  turnaround, not the board's: the meaningful measurement here is where
  loss begins, not the rate achieved - and at that traffic. Pumped
  harder, some 124 kB/s each way by `brio stress`, the 2 Mbaud echo's
  interrupt receiver overruns in hardware in about half the runs, one to
  three bytes, with the DMA transmitter beside it as in the duplex
  record above.)
- **At 115200 the console alone costs 11% of the CPU** through the
  per-byte interrupt path and 6% with DMA on both directions - the shape
  erratum 1.10.4 refuses - worth knowing, since every bench suite
  prints.
- **Erratum 1.10.4 can turn the transmit channel into a writer of the
  SERCOM's own registers** - the measurement that makes the two-engine
  shape a compile error. A both-engines echo that stopped answering,
  halted over SWD, sat in SERCOM5_Handler re-entered without end,
  INTENSET reading 0xAE (ERROR, RXBRK, RXS, RXC and TXC armed - a byte of
  the stream) and TXC, which the engined transport never serves,
  standing. The transmit channel's write-back held BTCTRL 0x0809 - the
  RECEIVE descriptor's, destination incrementing - with BTCNT 17 and DATA
  as both addresses, while its first-descriptor slot was intact: running
  that live copy, the channel read received bytes from DATA and wrote
  them at "end minus remaining", up through the registers below DATA.
  The dead-block predicate cannot see this (the block is running), and
  nothing at the channel level undoes it; the scribbled INTENSETs and
  the wedged transmitter of the twenty-run record above are the same
  walk caught short of a storm.
- **The dead-block predicate, asked of the engine alone, fired on live
  blocks.** Tested as the engine's `busy()` and the two flags, unmasked,
  it abandoned blocks in nine runs of twenty of letter h's 2 Mbaud echo,
  up to four in one - with the transmit engine's the only channel in the
  image, so with no erratum in reach. Instrumented, every firing caught
  found the channel DISABLED with nothing pending - a block already
  over, read as in flight across its completion while the receive
  interrupt entered every 240 cycles. Asked of the channel - enabled, no
  PEND, no BUSY - and decided under the mask, it fired in none of twenty.
- **The kick, measured and retired.** A scratch probe started 400
  messages a run through the transmit engine in four shapes - after a
  full drain at once, after 30 us, after 1.8 ms of standing DRE, and
  back to back - reading CHSTATUS and DRE right after each enable: over
  3376 block starts the first beat (and the second, the first byte gone
  straight to the idle shifter) had ALWAYS landed by the next register
  read, never once a block waiting with nothing pending, and with no
  kick at all every message arrived intact, no stall and no fault. With
  the kick and `write_bulk()` built `[[gnu::flatten]]` - which brings the
  DRE read into the first beat's window - `test_samc_dma`'s letter j lost
  the second byte of a line in four runs of four; without it, 708 lines
  of 708 in four runs. The receive re-arm's kick did the same in reverse
  once the engine's start had become fast enough to read RXC inside the
  first beat: the DMA receiver counted 8387 and 8396 bytes for 8384 sent
  (beats reading DATA twice); without it, 8384 and byte-exact. Reading
  CHSTATUS before the kick would narrow that window and not close it - a
  beat can start between the read and the kick - so neither direction
  kicks.

## Not covered yet

Driver gaps (not built):
- A BULK RECEIVE PATH THAT PACES ITSELF. `read_bulk()` exists, but the
  RX engine only publishes what `harvest()` takes, and how often to call
  it is left entirely to the port owner - which at 3 Mbaud means every
  hundred microseconds or so. Nothing in the driver helps a caller get
  that right, and getting it wrong loses bytes.
- A RECEIVE PATH WITH NO GAP AT ALL. The engine is idle between a block
  filling and the next harvest re-arming it, and everything that arrives
  in that window is lost. Two descriptors alternating on one channel
  would close it; linked descriptors are legal on this silicon revision
  but sit inside erratum 1.10.4's blast radius, so the shape is named
  and not built. A caller that cannot afford the gap should take the
  interrupt receiver, which has none.
- DMA ON BOTH DIRECTIONS OF ONE PORT: refused, erratum 1.10.4 ("The
  optional DMA engines"); the duplex bulk link is the transmit engine
  beside the interrupt receiver, lossless to 1 Mbaud (letter h). Born
  again only on a silicon revision the erratum's matrix leaves out.
- Within USART: fractional and 3x-arithmetic baud, synchronous mode
  and XCK, RTS/CTS handshaking, RS-485/TE, LIN, IrDA, collision
  detection, auto-baud, start-of-frame/RXS wake, 9-bit data uses,
  DBGCTRL policy (1.17.4: DBGSTOP does not actually halt
  transmission - the field is exposed, the erratum named).
- A per-package pad table (which pins reach which pads): the same
  device-table job [port.md](port.md) leaves open, born with its first
  user.

Implemented but not bench-verified:
- The console's CPU share with ONE engine at 115200 (the 6% above is
  the refused two-engine shape's): `serial_speed`'s occupancy, on its
  transmit-engine transport.
- `rebase()` (no dynamic clock exists on this target to drive it);
  nine-bit frames (this transport's rings are bytes); the USART
  personality on instances other than SERCOM5 (SERCOM1 and SERCOM3 run
  the SPI and the I2C personalities on silicon, [spi.md](spi.md) and
  [i2c.md](i2c.md), the USART only the console's instance); `release()`
  beyond the suite's own transport handovers.
