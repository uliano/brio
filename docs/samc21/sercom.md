# SERCOM - USART mode (SAM C21)

Documents of record: SAM C20/C21 data sheet DS60001479M - SERCOM
common ch. 30 (the baud generator, 30.6.2.3 table 30-2), USART
ch. 31 (transmission and reception 31.6.2.5-31.6.2.6, the loop-back
31.6.3.8, start-of-frame detection 31.6.3.9, the DMA and interrupt
requests 31.6.4, INTFLAG and STATUS 31.8.8-31.8.9) - and errata
DS80000740S items 1.17.15 and 1.17.16, both encoded in code (1.17.4 and
1.17.14 are named where their fields live), and 1.10.4 (the DMAC's,
[dmac.md](dmac.md)). Driver: `samc21/sercom.hpp` (`Sercom<n>` resource +
`Uart<n, pads, rx, tx, TxEngine>` task, the one engine slot - the
transmit one - optional). The family fixture is `test/family_samc21/sercom.cpp` plus
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
  DATA): DECLINED - this stratum drives the DMAC for one user, and that
  user is the transmitter ([dmac.md](dmac.md): the transport's own
  transmit and receive pair was measured writing this SERCOM's
  registers under erratum 1.10.4).
- AN IDLE-LINE FLAG, A RECEIVER TIME-OUT: NOT IN THIS SILICON - the
  receiver's interrupts are RXC, RXS, RXBRK and ERROR (31.6.4.2), and no
  counter of bit times after the last stop bit exists. The interrupt
  receiver needs neither: it has an edge on every character.
- START-OF-FRAME DETECTION (31.6.3.9, CTRLB.SFDE, the RXS interrupt):
  DECLINED for the edge - it says a character is beginning, not that a
  run has ended; its use is the wake from standby, a power chapter's.
- RXBRK: the break of the LIN and auto-baud formats (31.6.3.4-31.6.3.5),
  which are not built.
- THE ERRORS: STATUS's PERR, FERR and BUFOVF clear by being written
  (31.6.2.6.2, 31.8.9) - NO READ OF DATA, so a clear never takes a byte
  a channel was owed. The interrupt receiver reads STATUS before each
  character (31.8.11) and drops the hit ones. The ERROR interrupt is not armed (erratum 1.17.15).
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
- **`Uart<n, pads, rx_size, tx_size, TxEngine>`** - the
  task: two SPSC rings (lock-free at any size here - `atomic_width` 4
  - so 64/256 are console-class defaults, not a ceiling),
  `init(clock, baud, format)` speaking hertz off the clock tag,
  `isr()` as the ONE handler body honouring the edge-return contract
  (true on the RX ring's empty-to-non-empty transition - the kernel
  wakeup), taking every level of the receive buffer an entry and
  calling nothing on a clean character's path (the body, the ring's
  verbs and the register accesses all inline, so a handler placed in
  `.ram_text` runs that path from SRAM whole, [platform.md](platform.md);
  a receive error and a full ring, the rare paths, are calls out of
  line, which keep the clean path's registers: inline, the gap marks
  spilled the character to the stack), try-semantics `write_byte`
  and `write_bulk` (a run: as
  many as fit, its first byte pushed and DRE armed before the rest is
  copied and armed again behind it, or with the engine the run queued
  whole and the engine pumped once - the verb `print` hands every
  string and number), `read_byte`, `read_bulk` and `read_span`/`consume` (the
  receive run in place), `tx_idle()` - THE WIRE IDLE: nothing queued, no
  block in flight and TXC, the last stop bit gone (a port that has sent
  nothing since `init()` is idle too: TXC is clear out of reset) -, error
  counters, `rx_skips()` (THE SKIP EPOCH, util/stream.hpp's
  `SkippingSource`: never cleared - neither by `clear_errors()` nor by
  `init()` -, it moves whenever the line carried a character the ring
  will not deliver clean, between the run before and the run after: the
  receive ring's skips (a `SkipRing`, modulo 2^8), one at the consumer's
  look after a character dropped on a full ring, carrying BUFOVF or
  dropped for a frame or parity error - the look discarding what the
  ring held. A step an event, not a character: the hardware reports an overflow, not how
  many it lost. Moved on those rare paths alone, nothing on a clean
  character's),
  `rebase(hz)` for the day a dynamic clock exists, `set_baud(hz,
  baud)` (a new rate under the running port, once TX is idle),
  `release()` - the engine first, then the SERCOM RESET before its
  clocks are gated (CTRLA.SWRST, 31.6.2.2: a gated instance left enabled
  holds its transmit request and hands the next channel bound to its
  trigger a stray beat, [dmac.md](dmac.md)). Init
  order is deliberate: clocks, reset, configure,
  enable, and only THEN the pads to the SERCOM - the transmitter
  idles high before the pad leaves PORT, so no glitch start bit
  reaches the wire. `ByteTransport` and `ClockUser` are
  static_asserted. The engine slot defaults to `NoDmaEngine` - see "The
  optional transmit engine" below.
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
- **The resource's DMA constants** - `Sercom<n>::dma_tx_trigger()`
  (table 25-2's code, read off the device header per instance beside the
  GCLK id and APB mask) and `data_address()`, the one register the
  transmit engine writes.

## The optional transmit engine

An application that wants the transmitter on the DMAC includes
`samc21/dmac.hpp` alongside this header and names the engine in the
Uart's one slot; the receiver keeps its interrupt:

```cpp
using Serial = brio::Uart<5, console_pads, 64, 256, brio::DmaTxEngine>;

extern "C" void DMAC_Handler() { Serial::dma_isr(); }
```

The engine is a POLICY, not a feature of the task:

- **Zero when absent.** `NoDmaEngine` (the default) is a tag with
  `present = false`; every engine branch sits behind `if constexpr`, so
  a Uart that names no engine carries no test, no state and no code.
  sercom.hpp never includes dmac.hpp, so a program with a serial port
  does not carry the controller.
- **The trigger replaces the interrupt.** DRE is the SAME condition as
  the DMA trigger, so the engined transmitter does not arm DRE - both
  armed would serve every byte twice.
- **ONE ENGINE, ONE CHANNEL, ONE OWNER AT A TIME.** The DMAC is this
  stratum's for the Uart's transmitter alone (erratum 1.10.4,
  [dmac.md](dmac.md)): there is no receive slot - naming one is a
  compile error (`neg/uart_receive_engine_slot.cpp`) - and the engine
  is channel 0. Two Uarts naming it are two owners of one channel, so
  `init()` claims it with the SERCOM's number and an `init()` that finds
  it held by another transport PANICS (the breadcrumb's context names
  the refused SERCOM, dmac.md); `release()` gives the claim back, so
  transports may take the engine in turn. The engine brings the DMAC
  block up itself at `init()`: nothing else in the program owns it.
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
  (letter u, below). THE MASK COVERS THE CLAIM: a block start (from
  `write_byte`/`write_bulk` in main context or from the completion in
  the handler) takes the engine's reservation - a test-and-set of its
  busy flag under PRIMASK - and reads the run and programs the channel
  unmasked, because no completion of an engine that has no block in
  flight can land under it. The block itself is three stores into the
  descriptor slot and one store of the channel's enable; an engined
  transmit ring is at most 65535 bytes, a run being one block and BTCNT
  sixteen bits (refused at compile time). A bus error on a block (TERR,
  the channel disabled by the silicon, 25.6.2.8) is a block lost:
  abandoned, counted in `dma_faults()`, the next run started.
- **THE STANDING REQUEST, AND WHY NOTHING KICKS.** DRE is a LEVEL -
  "my transmit buffer is free" (31.6.4.1) - and the channel serves it as
  it finds it: standing when the channel is enabled, or when the claim
  selects the trigger onto it, it fires the block's first beat at once.
  So the enable fires the first beat by itself, and a software kick on
  top of it is a SECOND beat when the first has started (PEND clears as
  a beat starts, 25.8.23) - measured: a transmit kick doubled a byte
  into a full DATA (Bench findings). The channel keeps nothing across
  its own disable: a DRE that rose and fell while it was disabled fires
  no beat at the next enable ([dmac.md](dmac.md), "A released
  requester").
- **THE DEAD-BLOCK PREDICATE, ON THE REFUSED-BYTE PATH.** A channel
  that waits ENABLED with no trigger pending and no beat moving while
  DRE - its trigger - and TXC both stand cannot be running, so the
  block is abandoned and counted in `dma_faults()`. What killed blocks
  that way, measured, was erratum 1.10.4's corruption by a concurrently
  triggered channel; with one channel in the image it has no known
  cause, and every suite judges `dma_faults()` zero. The channel is
  asked and not the engine's own "in flight", which stays true from a
  block's last beat until its completion is heard and is true again for
  a block the handler has just started: with main context preempted for
  a character time across a completion, that test abandoned live blocks
  at 2 Mbaud with ONE channel in the image (Bench findings). The three
  readings are taken under the mask.
- **A REFUSED BYTE STILL NUDGES.** `write_byte()` returning false is
  the state in which nothing is draining the ring, and `print()`
  answers that false by trying again for ever - so the refusal path
  arms DRE (or pumps the engine) instead of returning silently, and
  `write_bulk()` does the same when nothing fitted. An EMPTY run
  through `write_bulk()` is a legal call for exactly this: it queues
  nothing and takes the same path, which is how a loop waiting for the
  ring to drain still gives the transport its push.

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

The transmitter on the engine, the receiver on its interrupt:

```cpp
using Serial = brio::Uart<5, console_pads, 64, 256, brio::DmaTxEngine>;

extern "C" void SERCOM5_Handler() {     // the receiver
    if (Serial::isr()) { brio::post<SerialLines>(brio::RxActivity{}); }
}
extern "C" void DMAC_Handler() { Serial::dma_isr(); }   // the transmit blocks
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
  exactly as designed).

**THE TWO SHAPES, BYTE FOR BYTE** (suite `test_samc_uart` with
`brio stress` at the other end; the pattern is a 32-bit
xorshift both ends generate, so a lost byte is located and not merely
counted). Interrupt on both directions, or the DMA transmitter beside
the interrupt receiver, as an echo of 8384 bytes at 115200 in a 1.2 s
window:

  | transport      | received    | notes                                    |
  |----------------|-------------|------------------------------------------|
  | irq TX + irq RX| 8384        | byte-exact both ways                     |
  | DMA TX + irq RX| 8384        | byte-exact both ways, 20 of 20, no block abandoned |

- **The interrupt receiver's edge and its skip, on the loop** (letter
  r): two polled bursts of 256 at 1 and at 3 Mbaud, each published by
  ONE edge from the vector, 512 of 512 byte-exact, no loss; then a
  consumer held back at 1 Mbaud - 700 sent into the 511 the ring holds:
  511 held, 189 counted in `rx_overruns`, no hardware overrun, the skip
  epoch unmoved until the consumer looks; its first look finds the ring
  discarded and the epoch moved by exactly one, and the next 128 arrive
  exact.
- **An eager and a lazy consumer** (letter n, a host sink at 115200, the
  ring read in place a run a look): eager, 10688 delivered with no gap;
  lazy - 150 ms looking, 100 ms held, five holds each overflowing the
  ring - five gaps, every one announced by the skip epoch before the run
  after it, no silent join.
- **The claim and its panic** (letters d and x): `claim()` refuses a
  second owner while the first holds the engine and grants it once
  `release()` has run; a second engined `init()` while the loop's
  transport holds the engine stops the board, and the next boot reads
  the breadcrumb: code 2 (`assert_failed`), context 0xD6 - SERCOM5, the
  refused owner.

- **The duplex link at speed**, the transmit engine beside the
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
  than a 160-cycle frame (letter u's uart.tx: x 2.05 at 3 Mbaud), so the
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
  to 1 Mbaud (x 1.02) and not at 3 Mbaud (x 2.05, 146 kB/s: the entry
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
  entry a character up to 1 Mbaud, 207-218 cycles between the bench's
  stamps, and at 3 Mbaud 0.34 to 0.37 entries a character (two or three
  levels an entry, 110-116 cycles a character - the bench's own stamps
  on every vector, the tick's included, cost it one character of 256 in
  five runs of six; the suite, unmetered, none) where one level an entry
  loses most of a burst (245 of 256, 53 overruns, measured).
- **The edge's latency** (letter u's `uart.edge`, the cycles from the
  sender's TXC to the ring holding the burst's last byte): the
  interrupt receiver 424 cycles after a pended RXC (the handler's entry
  and body, the stamp's own 65 inside).
- **The data sheet's own receive sequence, beside it** (a scratch
  program: a bare handler reading RXC, STATUS and DATA, one character an
  entry, 31.6.2.6, on the same loop and meters): 154 cycles a character
  up to 1 Mbaud against this driver's 207-218 - a gap of some 60 cycles:
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
  on the receiver's stop sample): 126 delivered in order and 130
  counted - exactly the characters whose parity bit is zero - three runs
  of three: STATUS clears by being written, never by a read of DATA.
- **The skip epoch, provoked**: letter s's 130 frame errors moved it
  130 times, once a dropped character, the consumer reading faster than
  the characters land so that every skip found the ring empty (three
  runs of three); the echo of letter h at 2 Mbaud moved it once for its
  one hardware overrun.
- **The console** (`console`, the interrupt transport under the line
  assembler, a 64-byte receive ring at 115200): HELP, ERR and forty
  `LED TOG` lines back to back answered forty OKs, every counter zero;
  forty HELP lines back to back - each reply eight times its command,
  printed blocking from the thread that drains the ring - filled the
  ring, 111 characters dropped and counted in `rx_overruns`, 18 replies.
- **The console's print** enters 139 cycles a byte between the bench's
  stamps (`bench_samc` letter p, 4096 bytes at 115200: 4097 SERCOM5
  entries), its transmit and receive paths inline in the vector.
- **The vector in SRAM** (`bench_samc_ram`, the handlers in `.ram_text`,
  [platform.md](platform.md)): with no call left on a clean
  character's path, either direction, the whole entry runs from SRAM, a
  third off every figure - the print 95 cycles an entry, the loop's
  transmitter 123 and its receiver 133 a character, and at 3 Mbaud the
  transmitter x 1.40 where it is 2.05 from the flash, the receiver 0.62
  entries a character with no overrun.
- **Erratum 1.10.4 can turn the transmit channel into a writer of the
  SERCOM's own registers** - the measurement that leaves the Uart with
  no receive slot ([dmac.md](dmac.md)). A both-engines echo that
  stopped answering, halted over SWD, sat in SERCOM5_Handler re-entered without end,
  INTENSET reading 0xAE (ERROR, RXBRK, RXS, RXC and TXC armed - a byte of
  the stream) and TXC, which the engined transport never serves,
  standing. The transmit channel's write-back held BTCTRL 0x0809 - the
  RECEIVE descriptor's, destination incrementing - with BTCNT 17 and DATA
  as both addresses, while its first-descriptor slot was intact: running
  that live copy, the channel read received bytes from DATA and wrote
  them at "end minus remaining", up through the registers below DATA.
  The dead-block predicate cannot see this (the block is running), and
  nothing at the channel level undoes it. Over twenty such echoes the
  erratum struck in nineteen: blocks abandoned, INTENSET scribbled, the
  transmitter wedged, and once the board silent.
- **The dead-block predicate, asked of the engine alone, fired on live
  blocks.** Tested as the engine's `busy()` and the two flags, unmasked,
  it abandoned blocks in nine runs of twenty of letter h's 2 Mbaud echo,
  up to four in one - with the transmit engine's the only channel in the
  image, so with no erratum in reach. Instrumented, every firing caught
  found the channel DISABLED with nothing pending - a block already
  over, read as in flight across its completion while the receive
  interrupt entered every 240 cycles. Asked of the channel - enabled, no
  PEND, no BUSY - and decided under the mask, it fired in none of twenty.
- **THE RATE ON THE WIRE, which the loop cannot tell** (its receiver
  samples on the divisor its transmitter shifts on): every loop letter
  and every rate the suite sets times a run of frames through the
  transmit engine - a hundredth of a second of frames, 16 at least and
  1024 at most, the receiver unserved - from the first block's start to
  TXC on the ticker's cycle count, against the run's wire time at the
  rate ASKED. The ruler and the generator share OSC48M, so the reading
  judges the divisor. Letter a's ladder: 9600 baud 998 to 999
  thousandths of the wire, 115200 to 2 Mbaud 999 to 1000, 3 Mbaud 1002
  to 1003 (the run's fixed cost of some tens of cycles in 163840);
  letter b's frames at 115200, from 8N1's ten bits to 8O2's twelve and
  5N2's eight, 999 to 1000; letters q, r, t and w at their own rates the
  same, 999 to 1003. The window
  is -1 % to +3 % ([../design/overview.md](../design/overview.md), "A
  loop proves the bytes, never the rate"), and the divisor's rounding is
  0.14 % at 9600 and less above.
- **The engine's level, read back** (letter d): CHCTRLB.LVL 2 with
  CTRL.LVLEN2 alone set ([dmac.md](dmac.md)).
- **The kick, measured and retired.** A scratch probe started 400
  messages a run through the transmit engine in four shapes - after a
  full drain at once, after 30 us, after 1.8 ms of standing DRE, and
  back to back - reading CHSTATUS and DRE right after each enable: over
  3376 block starts the first beat (and the second, the first byte gone
  straight to the idle shifter) had ALWAYS landed by the next register
  read, never once a block waiting with nothing pending, and with no
  kick at all every message arrived intact, no stall and no fault. With
  the kick and `write_bulk()` built `[[gnu::flatten]]` - which brings the
  DRE read into the first beat's window - a letter printing lines through
  the engine lost the second byte of a line in four runs of four;
  without it, 708 lines of 708 in four runs. Reading CHSTATUS before the
  kick would narrow that window and not close it - a beat can start
  between the read and the kick - so nothing kicks.

## Not covered yet

Driver gaps (not built):
- A RECEIVE ENGINE: declined - this stratum drives the DMAC for one
  user, the transmitter (erratum 1.10.4, [dmac.md](dmac.md)); the
  interrupt receiver takes every level of the buffer an entry and keeps
  3 Mbaud.
- THE TRANSMIT ENGINE ACROSS A STANDBY: its channel has RUNSTDBY = 0,
  which 25.6.7 makes software's to suspend before standby; nothing does
  (dmac.md). Born with the first program that sleeps to standby with an
  engined Uart.
- Within USART: fractional and 3x-arithmetic baud, synchronous mode
  and XCK, RTS/CTS handshaking, RS-485/TE, LIN, IrDA, collision
  detection, auto-baud, start-of-frame/RXS wake, 9-bit data uses,
  DBGCTRL policy (1.17.4: DBGSTOP does not actually halt
  transmission - the field is exposed, the erratum named).
- A per-package pad table (which pins reach which pads): the same
  device-table job [port.md](port.md) leaves open, born with its first
  user.

Implemented but not bench-verified:
- The console's CPU share at 115200, through the interrupt transport
  and with one engine, and the per-byte plateau through the bridge:
  `serial_speed`'s occupancy and throughput, whose host side - the
  checker of every byte at every rate - is not in `cli/`.
- `rebase()` (no dynamic clock exists on this target to drive it);
  nine-bit frames (this transport's rings are bytes); the USART
  personality on SERCOM0, 2, 3 and 4 (SERCOM5 runs the console, SERCOM1
  the loop's letters; SERCOM3 runs the I2C personality on silicon,
  [i2c.md](i2c.md)).
