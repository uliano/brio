# SERCOM - USART mode (SAM C21)

Documents of record: SAM C20/C21 data sheet DS60001479M - SERCOM
common ch. 30 (the baud generator, 30.6.2.3 table 30-2), USART
ch. 31 (transmission and reception 31.6.2.5-31.6.2.6, the loop-back
31.6.3.8, start-of-frame detection 31.6.3.9, the DMA and interrupt
requests 31.6.4, INTFLAG and STATUS 31.8.8-31.8.9), the DMAC's
write-back 25.6.2.5 - and errata DS80000740S items 1.17.15 and 1.17.16,
both encoded in code (1.17.4 and 1.17.14 are named where their fields
live), and 1.10.4 (the DMAC's, below). Driver: `samc21/sercom.hpp` (`Sercom<n>` resource +
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

**A receiver on its own transmitter's pad is a loop with no wire**
(31.6.3.8: RXPO and TXPO naming one pad, "the loop-back is through the
pad, so the signal is also available externally"). `UartPads` admits
the pair when it names one pin for both directions (`uart_pads_loop_back`);
the suite and the bench run SERCOM1 that way on PA16, which nothing is
wired to, at every rate the generator makes.

**The receive side, item by item** - what the chapter offers for
taking a burst, and what this driver does with each:

- THE TWO-LEVEL RECEIVE BUFFER (31.6.2.6): USED, every level an entry -
  the handler reads RXC again after each character and leaves when it
  reads clear, so one entry serves the two characters a late entry
  finds, and the receiver keeps a 3 Mbaud stream (letter t).
- RXC AS A DMA REQUEST (31.6.4.1, cleared by the channel's read of
  DATA): USED by the optional receive engine, which fills half of the
  ring a block and whose completion is the run's edge (below).
- AN IDLE-LINE FLAG, A RECEIVER TIME-OUT: NOT IN THIS SILICON - the
  receiver's interrupts are RXC, RXS, RXBRK and ERROR (31.6.4.2), and no
  counter of bit times after the last stop bit exists. So the tail of a
  run under the engine - the bytes after its last filled half - has no
  edge of its own: the owner asks (`harvest()`).
- AN IDLE DETECTOR BUILT AROUND THE SERCOM - the RX pad's edges through
  EIC and EVSYS into a TC retriggered by each edge, its overflow the
  idle line: DECLINED. It costs a TC, an event channel and an EIC line
  per port, the EIC line on the RX pin's own EXTINT; the bulk receiver it
  would serve is the engine's, whose runs fill its halves, and the
  console - the burst receiver - is the interrupt receiver, which has an
  edge on every character and ends a line at its terminator.
- START-OF-FRAME DETECTION (31.6.3.9, CTRLB.SFDE, the RXS interrupt):
  DECLINED for the edge - it says a character is beginning, not that a
  run has ended; its use is the wake from standby, a power chapter's.
- RXBRK: the break of the LIN and auto-baud formats (31.6.3.4-31.6.3.5),
  which are not built.
- THE ERRORS: STATUS's PERR, FERR and BUFOVF clear by being written
  (31.6.2.6.2, 31.8.9) - NO READ OF DATA, so a clear never takes a byte
  a channel was owed. The interrupt receiver reads STATUS before each
  character (31.8.11) and drops the hit ones; the engine reads it once
  a run (below). The ERROR interrupt is not armed (erratum 1.17.15).
- IBON (31.6.2.6.2): BUFOVF raised at once instead of travelling with
  the data - exposed (`SercomUartConfig::immediate_overflow`), off by
  default: the counter is per event either way.
- TXC (31.8.8): `tx_idle()`'s last clause - set when the stop bit has
  left with nothing in DATA, cleared by any write to DATA, a channel's
  beat included (measured, letter q).

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
  wakeup), taking every level of the receive buffer an entry and
  calling nothing (the body, the ring's verbs and the register
  accesses all inline, so a handler placed in `.ram_text` runs from
  SRAM whole, [platform.md](platform.md)), try-semantics `write_byte`
  and `write_bulk` (a run: as
  many as fit, its first byte pushed and DRE armed before the rest is
  copied and armed again behind it, or with an engine the run queued
  whole and the engine pumped once - the verb `print` hands every
  string and number), `read_byte`, `read_bulk` and `read_span`/`consume` (the
  receive run in place), `tx_idle()` - THE WIRE IDLE: nothing queued, no
  block in flight and TXC, the last stop bit gone (a port that has sent
  nothing since `init()` is idle too: TXC is clear out of reset) -, error
  counters,
  `rebase(hz)` for the day a dynamic clock exists, `set_baud(hz,
  baud)` (a new rate under the running port, once TX is idle),
  `release()`. Init
  order is deliberate: clocks, reset, configure,
  enable, and only THEN the pads to the SERCOM - the transmitter
  idles high before the pad leaves PORT, so no glitch start bit
  reaches the wire. `ByteTransport` and `ClockUser` are
  static_asserted. The two engine slots default to `NoDmaEngine` -
  see "The optional DMA engines" below.
- **THE COPY into the transmit ring and out of the receive one**
  (`write_bulk`, `read_bulk`): an inline byte loop, six instructions a
  byte - about ten cycles behind the flash's two wait states -, or the
  runtime's `memcpy` for a run of `uart_copy_crossover` bytes (20) or
  more whose two ends share their alignment in the word, where its word
  path moves the run at about 1.3 cycles a byte past a fixed cost of
  some 170 cycles. Across a misalignment `memcpy`'s byte path is the
  same loop out of line and a call dearer (eleven cycles a byte
  measured), so it is not called there. Measured with `bench_samc`'s
  letter u, below.
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
- **TX drains the ring in blocks of HALF THE RING at most.**
  `write_byte`, `write_bulk` and `print` are unchanged; the engine is
  handed the ring's contiguous run (`read_span`, design/ring.md), cut at
  half the ring, and its completion interrupt consumes exactly the block
  it carried and starts the next - a wrapped ring goes out in two
  blocks. The cut is what keeps a producer longer than the ring on the
  wire: a block's slots come back only at its completion, so a block of
  the whole ring left the producer nothing to refill until the wire had
  gone idle - measured, 4096 bytes through a 2048-byte ring at 3 Mbaud
  ran at x 1.06, the second copy in the gap; cut at half, x 1.00
  (letter u, below). The app's DMAC handler routes completions
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
- **RX FILLS HALF THE RING A BLOCK, AND THE BLOCK'S COMPLETION IS THE
  EDGE.** The engine is handed the ring's free run cut at half the
  ring; when the block fills, the DMAC's vector - the app's binding
  calls `dma_isr(channel)` - publishes the run (the completion says
  every beat landed: `DmaRxEngine::complete()` counts it with no
  write-back read and no suspend), hands the channel its next run IN
  THE SAME HANDLER, and returns the empty-to-non-empty edge, so the
  glue that posts `RxActivity` on `isr()`'s true posts it on
  `dma_isr()`'s too. The channel waits for no owner between blocks: it
  is idle from its last beat to that handler's re-arm, which the
  receiver's two levels cover - a stream at 3 Mbaud crosses the block
  boundaries byte-exact (letter r) - and a consumer told at every half
  drains one while the channel fills the other. The edge costs 490
  cycles of handler (549 between the bench's stamps, letter u) once a
  half ring: 2.1 cycles a byte on a 256-byte block.
- **THE TAIL IS THE OWNER'S ASK.** A run that stops short of its block's
  end has no edge (no idle detector and no time-out in this silicon,
  "The receive side, item by item" above), so `harvest()` stays a
  public verb: it suspends the channel, reads the validated write-back
  (erratum 1.10.4, dmac.md), publishes what landed and re-arms a
  channel that is not running, returning the same edge. It runs whole
  under the mask - the ring's producer side is the handler's too - about
  700 cycles, 44 us when the suspend never lands (dmac.md). When to ask
  is the owner's knowledge: at the end of a message of known length, a
  terminator's expected time, or a lazy clock for a stream of unknown
  length; an EAGER clock costs - measured, an ask every 50 us lost two
  to five characters at a block boundary in every run, uncounted, where
  an ask every 2 ms lost none (letter n, Bench findings). A consumer
  that releases slots (`consume`, `read_byte`, `read_bulk`) re-arms an
  engine a full ring left idle.
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
  RXC, STATUS is read once a run - at its completion or an owner's ask
  - and its errors are counted against the run, not a byte; the hit
  characters are delivered with the rest, their data bits as received.
  No byte is lost to the count: STATUS clears by being written, never
  by a read of DATA (letter s: 256 characters, 130 of them frame
  errors, all 256 delivered byte-exact under the engine). A protocol
  with its own framing does not care; a console that wants exact
  frame-error attribution should not take an RX engine.

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

A bulk receiver on the engine takes its edge from the DMAC's vector and
asks for a tail on its own clock:

```cpp
using Stream = brio::Uart<5, console_pads, 512, 64,
                          brio::NoDmaEngine, brio::DmaRxEngine<1>>;

extern "C" void SERCOM5_Handler() { (void)Stream::isr(); }   // the transmitter
extern "C" void DMAC_Handler() {
    while (const auto irq = brio::Dmac::take_pending()) {
        if (Stream::dma_isr(irq->channel)) {     // a filled half of the ring
            brio::post<Sink>(brio::RxActivity{});
        }
    }
}
// ... and where the owner knows a run should have ended:
//     if (Stream::harvest()) { brio::post<Sink>(brio::RxActivity{}); }
```

A loop with no wire, for a test: one pin named for both directions.

```cpp
constexpr brio::UartPads loop_pads{
    .tx = brio::SercomPad::pad0, .rx = brio::SercomPad::pad0,
    .tx_pin = {'A', 16, brio::PinFunction::c},
    .rx_pin = {'A', 16, brio::PinFunction::c},
};
using Loop = brio::Uart<1, loop_pads>;
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
  exactly as designed); the transmit engine beside two memory-to-memory channels sprayed for
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
  | irq TX + DMA RX| 8384        | byte-exact both ways: 32 edges from the engine's vector, the tail by an ask every 2 ms |

  The receive engine re-arms in its own completion handler, so a block
  boundary is no gap: byte-exact in every run, at 115200 through the
  bridge (letters g, p) and at 1 and 3 Mbaud on the loop, a polled
  sender keeping the wire full across two boundaries (letter r: 1300 of
  1300, two edges from the vector, no overrun). What does lose is an
  owner's ask on an EAGER clock (letter n): asked every 50 us - some
  twenty suspends a block - the stream lost two to five characters at a
  block boundary in every run, uncounted (first_bad a multiple of the
  block: 256, 512, 1024, 1280), where asked every 2 ms it lost none. The
  suspend that lands on a block's last beats is the suspect, its
  mechanism not isolated ("Not covered yet"); the engine's edge needs no
  ask, and the tail wants one, not a clock of them. It never receives
  MORE than was sent (it did, while the re-arm kicked: below).

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
  interrupt receiver, letter h: at 1 Mbaud 64000 bytes each way,
  byte-exact on the board's side every run, no hardware overrun; at
  2 Mbaud 105000, the receiver taking every level an entry and
  overrunning in hardware in about half the runs by one to three
  characters (counted in `hw_overruns`: another handler - the transmit
  engine's completion, the tick - held it past two frames), and no
  transmit block abandoned in any run of either rate. At 2 Mbaud the
  echo also finds its transmit ring full now and then - 0 to 18
  characters a window in six runs, counted in `dropped` - consistent
  with the board sending back slower than the bridge sends in: OSC48M
  is half a per cent off nominal on this board
  ([../boards/samc21j.md](../boards/samc21j.md)), and half a per cent of
  105000 is the ring's 511. The host's own view of the echo above 115200 is the
  bridge's and is not judged: the plain transport's echo at 1 Mbaud,
  byte-exact on the board, reached the host 635 bytes short.

- **Rates, through the interrupt transport, echoing:** 115200 and
  1 Mbaud are byte-exact; at 3 Mbaud the RECEIVER keeps the stream - no
  hardware overrun, every level an entry - and the echo loses in the
  software ring instead (`rx_overruns`), ACCOUNTED FOR rather than
  silent: the echo's transmitter feeds one character an entry, longer
  than a 160-cycle frame (letter u's uart.tx: x 2.15 at 3 Mbaud), so the
  receive ring fills behind it. On the loop, a polled sender at the
  wire's rate, the interrupt receiver takes 400 characters of 400 at
  3 Mbaud (letter t).
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
- **The transmitter's shapes, measured on the loop** (`bench_samc`
  letter u, 4096 bytes, the wall from the first call to the last stop
  bit): through the interrupt transport one DRE entry a byte, 194-198
  cycles between the bench's stamps (about 140 the body), so the wire is held
  to 1 Mbaud (x 1.02) and not at 3 Mbaud (x 2.15, 139 kB/s: the entry
  outlasts the 160-cycle frame); through the transmit engine x 1.00 at
  every rate, 298 kB/s at 3 Mbaud, six completions and 2 per cent of the
  CPU - the copy into the ring and the idle loop's turns. The engine's
  blocks are half the ring: a block of the whole ring left a producer
  longer than the ring waiting for the wire to go idle (measured, x 1.06
  at 3 Mbaud).
- **The copy into the ring** (letter u's `uart.copy`, `write_bulk()`
  timed at nine lengths and the four source alignments): the inline
  byte loop costs about ten cycles a byte behind two wait states, the
  runtime's `memcpy` about 170 cycles of call and tests and then 1.3 a
  byte when the two ends share their alignment, eleven when they do not.
  So a run of 20 bytes or more whose ends are co-aligned goes through
  `memcpy`, the rest through the loop: a 1024-byte run 1749 cycles
  co-aligned, 11566 otherwise.
- **The receiver's shapes, measured on the loop** (letter u, bursts
  of 16 and 256 from the transmit engine): the interrupt receiver one
  entry a character up to 1 Mbaud, 211-221 cycles between the bench's
  stamps, and at 3 Mbaud 0.34 to 0.37 entries a character (two or three
  levels an entry, 110-116 cycles a character - the bench's own stamps
  on every vector, the tick's included, cost it one character of 256 in
  five runs of six; the suite, unmetered, none) where one level an entry
  loses most of a burst (245 of 256, 53 overruns, measured). The
  receive engine: one completion per filled half of the
  ring, 549 cycles between the stamps - 2.1 a character on a 256-byte
  block -, none per character.
- **The edge's latency** (letter u's `uart.edge`, the cycles from the
  sender's TXC to the ring holding the burst's last byte): the
  interrupt receiver 424 cycles after a pended RXC (the handler's entry
  and body, the stamp's own 65 inside); the engine's completion 659 to
  766 cycles after TXC at 1 Mbaud and 115200 - within two frames at
  1 Mbaud (480-cycle frames), a fifth of one at 115200. A run that stops
  short of its block has no edge until the owner asks: 23000 to 30000
  cycles on the bench's once-a-tick ask.
- **The data sheet's own receive sequence, beside it** (a scratch
  program: a bare handler reading RXC, STATUS and DATA, one character an
  entry, 31.6.2.6, on the same loop and meters): 154 cycles a character
  up to 1 Mbaud against this driver's 211-221 - a gap of 60 cycles:
  the shared vector's question (INTENSET read beside INTFLAG: DRE is a
  condition), the level loop's last RXC read, and the ring's full test
  and the edge; at 3 Mbaud it lost 241 of 256 (115 error entries) where
  this driver's level loop kept them.
- **`tx_idle()` on the pad** (letter q, the loop's receiver read by the
  probe itself, its handler held off): the receiver's flag for the last
  character and TXC rise within one turn of the probe at every rate -
  either first -, and `tx_idle()` answers within that turn of the last
  byte, through the interrupt transport at 115200 and 1 Mbaud and
  through the engine at 115200, 1 and 3 Mbaud. A channel's beat into
  DATA clears TXC as a CPU write does, so an engined block never
  answers on a TXC its previous frame left.
- **No byte is taken by a clear** (letter s, the host sending 256
  characters at 8E1 into an 8N1 receiver - every zero parity bit lands
  on the receiver's stop sample): under the receive engine all 256
  delivered byte-exact with 130 frame errors in the stream, counted once
  for the run; under the interrupt receiver 126 delivered in order and
  130 counted - exactly the characters whose parity bit is zero - three
  runs of three.
- **The console's print** enters 139 cycles a byte between the bench's
  stamps (`bench_samc` letter p, 4096 bytes at 115200: 4097 SERCOM5
  entries), its transmit and receive paths inline in the vector.
- **The vector in SRAM** (`bench_samc_ram`, the handlers in `.ram_text`,
  [platform.md](platform.md)): with no call left on either path the
  whole entry runs from SRAM, a third off every figure - the print 92
  cycles a byte, the loop's transmitter 129 and its receiver 140 a
  character, and at 3 Mbaud the transmitter x 1.46 where it is 2.15
  from the flash, the receiver 0.6 entries a character with no
  overrun.
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
- AN EDGE FOR THE TAIL OF A RUN UNDER THE RECEIVE ENGINE. The engine's
  edge is its block's completion; the bytes after the last filled half
  are published when the owner asks. Declined because this silicon has
  no idle flag and no receiver time-out, and the idle detector built
  around the SERCOM (pad edges through EIC and EVSYS into a retriggered
  TC) costs a TC, an event channel and an EIC line per port for a
  receiver whose bursts the interrupt receiver serves ("The receive
  side, item by item").
- AN OWNER'S ASK THAT IS SAFE AT ANY CADENCE. Asked every 50 us, the
  engine lost two to five characters at a block boundary, uncounted
  (letter n); asked every 2 ms, none. The mechanism is not isolated: the
  suspect is the suspend landing on a block's last beats. The ask that
  needs no suspend - the DMAC's ACTIVE register, whose BTCNT is the
  active channel's live count while ABUSY stands (25.8.14) - is
  dmac.md's to build and measure.
- A RECEIVE PATH WITH NO IDLE BEAT AT ALL. The engine is idle from a
  block's last beat to its completion handler's re-arm, which the
  receiver's two levels cover at 3 Mbaud with no other handler longer
  than about two frames (320 cycles) beside it; a longer one - another
  owner's - can overrun it. Two descriptors alternating on one channel
  would close it; linked descriptors sit inside erratum 1.10.4's blast
  radius, so the shape is named and not built.
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
- The receive engine's re-arm by the consumer after a full ring left it
  idle (`resume_rx()` on `consume`, `read_byte`, `read_bulk`): no run of
  the suites fills the ring under the engine; a letter that holds its
  consumer back for a block would.
- `test_samc_dma`'s letter i - a burst typed by hand into the receive
  engine, its edge now from the vector and its tail from a tick-paced
  ask - needs a person at the keyboard.
- The console's CPU share at 115200, through the interrupt transport
  and with one engine, and the per-byte plateau through the bridge:
  `serial_speed`'s occupancy and throughput, whose host side - the
  checker of every byte at every rate - is not in `cli/`.
- `rebase()` (no dynamic clock exists on this target to drive it);
  nine-bit frames (this transport's rings are bytes); the USART
  personality on SERCOM0, 2, 3 and 4 (SERCOM5 runs the console, SERCOM1
  the loop's letters; SERCOM3 runs the I2C personality on silicon,
  [i2c.md](i2c.md)); `release()` beyond the suites' own transport
  handovers.
