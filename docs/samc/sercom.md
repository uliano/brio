# SERCOM - USART mode (SAM C21)

> **PROVISIONAL.** The asynchronous, internal-clock, 16x-arithmetic
> USART is implemented and bench-verified end to end - the console
> personality - including the two OPTIONAL DMA engines (transmit and
> receive, each compiling to zero when not named). The SERCOM's other
> three personalities (SPI host and client, I2C host and client) and
> the USART chapter's own long tail (fractional baud, synchronous
> mode, handshaking, LIN, IrDA, auto-baud) are declared, not built.
> The list is in "Not covered yet".

Documents of record: SAM C20/C21 data sheet DS60001479M - SERCOM
common ch. 30 (the baud generator, 30.6.2.3 table 30-2), USART
ch. 31 - and errata DS80000740S items 1.17.15 and 1.17.16, both
encoded in code (1.17.4 and 1.17.14 are named where their fields
live). Driver: `samc/sercom.hpp` (`Sercom<n>` resource +
`Uart<n, pads, rx, tx, TxEngine, RxEngine>` task, the engine slots
optional). The family fixture is `test/family_samc/sercom.cpp` plus
its negatives under `tools/check_samc.sh`.

## What the silicon does

**One interrupt vector for the whole instance.** DRE, TXC, RXC, RXS,
CTSIC, RXBRK and ERROR all share the instance's single NVIC line -
the one place where the AVR shape (two vectors, two ISR bodies) must
not be copied. And DRE is a CONDITION, not an event: it reads 1
whenever the transmit buffer is empty, which is most of the time, so
a handler acting on raw INTFLAG would run the transmit path on every
receive interrupt. The mask that matters is INTFLAG AND INTENSET.

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
Measured consequence of getting this wrong, kept as the cautionary
fact: every byte arrives exactly bit-reversed at the correct baud
(0x0D 0x0A 'S' reads back 0xB0 0x50 0xCA), with every other register
correct - the driver's `UartFormat` defaults the bit and a family
static_assert keeps it from regressing.

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
returns an optional rather than overloading zero. TXC, unlike the
AVR's, is exact: cleared by every DATA write and set only when the
shifter empties with nothing queued - "the last byte is on the wire",
no timing guesswork.

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
  `isr()` as the ONE handler body with the AVR's edge-return contract
  intact (true on the RX ring's empty-to-non-empty transition - the
  kernel wakeup), try-semantics `write_byte`, `read_byte`, error
  counters, `rebase(hz)` for the day a dynamic clock exists,
  `release()`. Init order is deliberate: clocks, reset, configure,
  enable, and only THEN the pads to the SERCOM - the transmitter
  idles high before the pad leaves PORT, so no glitch start bit
  reaches the wire. `ByteTransport` and `ClockUser` are
  static_asserted. The two engine slots default to `NoDmaEngine` -
  see "The optional DMA engines" below.
- **The resource's DMA constants** - `Sercom<n>::dma_rx_trigger()` /
  `dma_tx_trigger()` (table 25-2's codes, read off the device header
  per instance beside the GCLK id and APB mask - the family fixture
  static_asserts them against `samc/dmac.hpp`'s own spelling so the
  two cannot drift) and `data_address()`, the one register a transfer
  ever touches.

## The optional DMA engines

An application that wants DMA on a direction includes `samc/dmac.hpp`
alongside this header and names an engine with its channel:

```cpp
using Serial = brio::Uart<5, console_pads, 64, 256,
                          brio::DmaTxEngine<0>, brio::DmaRxEngine<1>>;
```

The engines are POLICIES, not features of the task:

- **Zero when absent.** `NoDmaEngine` (the default) is a tag with
  `present = false`; every engine branch sits behind `if constexpr`,
  so a Uart that names no engine carries no test, no state, no code -
  measured, not asserted: the release images of the engine-less apps
  are byte-identical to the ones built before the parameters existed.
  sercom.hpp never includes dmac.hpp (the engines live THERE), so a
  program with a serial port does not carry descriptor tables.
- **The trigger replaces the interrupt.** DRE for the transmitter and
  RXC for the receiver are the SAME condition as the DMA trigger, so
  whichever direction has an engine does not arm its interrupt -
  both armed would serve every byte twice. The two directions are
  independent: one engine is a legal shape (DMA on the bulk
  direction, the RXC interrupt's exact per-character error
  attribution on the other), and the two must name DIFFERENT channels
  (compile-time refused - a channel moves bytes one way).
- **TX drains the ring in blocks.** `write_byte`/`print` are
  unchanged; the engine is handed the ring's contiguous run
  (`read_span`, design/ring.md) and its completion interrupt consumes
  exactly the block it carried and starts the next - a wrapped ring
  goes out in two blocks. The app's DMAC handler routes completions
  with `dma_isr(channel)`, which answers false for channels that are
  not this transport's.
- **RX fills the ring's free run and is HARVESTED on the caller's
  clock.** A receive block completes only when the buffer fills -
  on an idle line, never - so arrival is not an event anyone is told
  about: `harvest()` suspends the channel, reads the validated
  write-back (erratum 1.10.4, see dmac.md), publishes the fresh
  bytes, and returns the same empty-to-non-empty edge `isr()` has, so
  the same kernel glue works. Pacing is WHOEVER OWNS THE PORT's
  policy - a kernel TimeEvent every few ticks is the shape brio
  expects - and each harvest costs ~10 us of masked interrupts.
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
- The DORD lesson above was measured, not deduced: right baud,
  bit-reversed bytes, every other register verified over SWD in one
  halt-and-dump.
- The enable-clears-TXEN/RXEN clause and the pull-down-only clause
  were both verified against the data sheet text verbatim - neither
  is folklore.
- The engined transport, live on the console's own SERCOM5: print()
  through the TX engine is byte-exact on the wire (a six-line banner
  went out as seven DMA blocks - the ring wrap served as two spans,
  exactly as designed); the RX engine with a tick-paced harvest
  served a typed burst unchanged with the RXC interrupt never armed;
  both engines at once survived the erratum-1.10.4 stress with zero
  violations on their own channels (the full account is in dmac.md).

**How fast the link really goes, and what it costs** (probe app
`serial_speed`, which reports throughput while the host checks every
byte). The measurements are of the BENCH LINK - an ADuM1201 isolator and
a CH340 bridge between the pads and the PC - as much as of the driver.

- **3 Mbaud works**, which is the generator's own ceiling at 48 MHz
  (16x oversampling, BAUD 0). A raw polled transmit - no ring, no
  interrupt, no DMA - moved 64 KB byte-exact at 299251 B/s, 99.75% of
  nominal. So neither the isolator nor the bridge is the limit.
- **2.5 Mbaud is a hole, not a ceiling**: it fails while both 2 M and
  3 M are byte-exact. The bridge's divisor arithmetic has no exact
  2.5 M and the nearest is some 4% off, outside what a UART tolerates.
  A failure at one rate says nothing about the rate above it.
- **The per-byte API is what limits a fast link.** `write()` loops over
  `write_byte()`, and every byte pays a transport nudge - arming DRE, or
  `pump_tx()` with an engine. That plateaus at 98.4 kB/s (about 1 Mbaud
  equivalent) at EVERY rate from 1 Mbaud up, the wire idling while the
  CPU catches up. Fed this way the DMA engines are SLOWER than the
  interrupt, 57-64 kB/s at 92% CPU, because a pump_tx() per byte starts
  a block for one byte.
- **`write_bulk()` is what makes the engines worth having.** The same
  64 KB at 3 Mbaud: 169343 B/s at 75% CPU through the interrupt
  transport, and 297890 B/s - 99.3% of the wire - at 9% CPU through the
  engines. At 1 Mbaud the engines saturate the wire at 5% CPU against
  70% for the per-byte interrupt path.
- **With the engines, transmit is limited by the BAUD GENERATOR and
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
  loss begins, not the rate achieved.)
- **At 115200 the console alone costs 11% of the CPU** through the
  per-byte interrupt path and 6% through the engines - worth knowing,
  since every bench suite prints.

## Not covered yet

Driver gaps (not built):
- The SPI and I2C personalities - their own future drivers.
- A BULK RECEIVE PATH THAT PACES ITSELF. `read_bulk()` exists, but the
  RX engine only publishes what `harvest()` takes, and how often to call
  it is left entirely to the port owner - which at 3 Mbaud means every
  hundred microseconds or so. Nothing in the driver helps a caller get
  that right, and getting it wrong loses bytes silently.
- Within USART: fractional and 3x-arithmetic baud, synchronous mode
  and XCK, RTS/CTS handshaking, RS-485/TE, LIN, IrDA, collision
  detection, auto-baud, start-of-frame/RXS wake, 9-bit data uses,
  DBGCTRL policy (1.17.4: DBGSTOP does not actually halt
  transmission - the field is exposed, the erratum named).
- A per-package pad table (which pins reach which pads): the same
  device-table job the PORT driver leaves open.

Implemented but not bench-verified:
- **The RX engine in FULL DUPLEX.** Transmitting through the TX engine is
  flawless at every rate measured, but an echo that runs both engines at
  once loses most of the stream and can leave the transport wedged (a
  blocked `print()` on a TX ring that stops draining). `test_samc_dma`'s
  duplex letter passes, so either that suite's shape or the probe's
  misses the condition; it is not isolated.
- Frame formats other than 8N1 (parity, 5..7/9 bits, two stop);
  `rebase()` (no dynamic clock exists on this target to drive it);
  instances other than SERCOM5; `release()`; operation on the E/G
  variants (compile-checked only).
