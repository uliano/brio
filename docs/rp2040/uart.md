# UART (RP2040)

Documents of record: the RP2040 datasheet (build 3184e62), 4.2
(UART: the ARM PL011 - 4.2.2 functional description, 4.2.3 operation,
4.2.6 interrupts, 4.2.7 the programmer's model and the baud
calculation of 4.2.7.1, 4.2.8 the registers), 2.19.2 (table 279: the
pins), 2.15.3.1 (clk_peri). The driver is ARM's and not this chip's, so
it lives in the IP stratum: `brio/pl011/uart.hpp` holds the resource and
the byte transport ([../pl011/README.md](../pl011/README.md)), and
`brio/rp2040/uart.hpp` holds what this chip owes it - the pin table of
table 279, the `Rp2040Pl011` chip traits over `pin.hpp`, `resets.hpp`,
`clock.hpp`, `nvic.hpp` and `dma_engine.hpp`, and the PUBLIC NAMES
`Pl011<n>` (the resource) and `Uart<n, pins, ...>` (the task), this
chip's aliases of the two templates there. The reference suite:
`test_rp2040_serial` (the second instance under the loop-back for the
formats, the ladder, the FIFOs, a break, an overrun and bulk traffic; a
peer board on the cross link; the host at the console's other end).

## What the silicon does

Two ARM PL011s, clocked from clk_peri (UARTCLK) with their registers
on clk_sys. The baud rate divisor is UARTCLK / (16 x baud), a 16-bit
integer (UARTIBRD) plus a 6-bit fraction (UARTFBRD), so the reach is
UARTCLK/16 down to UARTCLK/(16 x 65535); the two registers take
effect on the NEXT write of the line control register, and LCR_H,
IBRD and FBRD are written with the UART disabled. Both FIFOs are 32
deep; the receive FIFO stores the framing, parity and break flags
WITH EACH BYTE (bits 8..10 of UARTDR), while the overrun flag (bit
11) is a LIVE condition - set while the FIFO is full and a frame
lands, cleared as soon as a read makes room - so the entries never
show it and UARTRSR's sticky OE is where an overrun is read
(measured). The loop-back bit UARTCR.LBE feeds the transmitter into
the receiver with the RX pad ignored. Eleven maskable interrupts
combine into one line per instance (UART0_IRQ, UART1_IRQ): receive at
the FIFO's trigger level, RECEIVE TIMEOUT when bytes wait and the
line has idled 32 bit periods, transmit when the FIFO falls THROUGH
its trigger level - "based on a transition through a level, rather
than on the level itself" (4.2.6.3), cleared by writing the FIFO
above the level or through UARTICR, so enabling it over an empty FIFO
raises nothing - and the four errors. The
modem inputs and DTR/OUT1/OUT2 exist in the block and reach no pad on
this chip; CTS and RTS do. The pins are fixed per instance under
function 2: UART0 transmits on GPIO 0, 12, 16, 28 and receives on 1,
13, 17, 29; UART1 on 4, 8, 20, 24 and 5, 9, 21, 25.

## Types and verbs

- `UartBits` (5..8), `UartParity`, `UartFormat` (bits, parity, 1 or 2
  stop bits), `uart_format_valid`.
- `UartPins` (`PinSel` tx and rx), `uart_tx_pin(n, pin)`,
  `uart_rx_pin(n, pin)`, `uart_pins_valid(n, pins)` - table 279 as
  constexpr facts; a pin set the table does not give the instance
  does not compile.
- `UartDivisor` (integer, fraction), `uart_divisor(hz, baud)` (nothing
  when out of reach), `uart_actual_baud(hz, d)`, `uart_min_hz(baud)`.
- `UartFlag` (UARTFR), `UartInterrupt` (one layout for IMSC/RIS/MIS/
  ICR), `UartDataError` (UARTDR's flags), `UartReceiveStatus`
  (UARTRSR's sticky four), `UartControl` (UARTCR), `UartLineControl`
  (UARTLCR_H), `UartTriggerField` (UARTIFLS), `UartBaudField` (the two
  divisor registers' masks), `UartDmaControl` (UARTDMACR),
  `UartFifoLevel` (the trigger levels in eighths).
- `Pl011<n>` - the resource: `reset`/`hold`/`released` (through the
  reset controller), `enable(on)` (UARTEN with TXE and RXE),
  `line_control(format, fifos)`, `divisor(d)` and `loopback(on)`
  (all three refused while enabled; the divisor followed by the
  latching LCR_H write), `break_send(on)` (live), `fifo_levels(rx,
  tx)` and their readback, `fifos_enabled`, the flags (`tx_full`,
  `rx_empty`, `busy`), `read_data`/`write_data`, the receive status,
  the interrupt mask/pending/clear trio through the atomic aliases,
  `dma_requests`, `irq()`.
- `Uart<n, pins, rx_size = 64, tx_size = 256, TxEngine, RxEngine>` -
  the task, every target's Uart surface: `init(clock, baud, format)`,
  `isr()` (true on the receive ring's empty-to-non-empty edge),
  `write_byte`, `write_bulk`, `read_byte`, `read_bulk`, `read_span`
  and `consume` (the receive run in place), `rx_pending`, `tx_idle`, `rebase(hz)`, `set_baud(hz, baud)`,
  `set_format(format)`, `loopback(on)` (each under the running port,
  after a drain), `min_hz_for`, `can_baud`, `actual_baud(hz)`,
  `release()`; the counters `rx_overruns` (ring), `frame_errors`,
  `parity_errors`, `break_errors` (a break entry carries FE too, and
  both are counted), `hw_overruns` (one per overrun event: UARTRSR's
  sticky OE on the interrupt receiver, the OE interrupt under an
  engine), `clear_errors`. The two engine slots take `dma.hpp`'s
  `DmaTxEngine` / `DmaRxEngine` ([dma.md](dma.md)): with a transmit
  engine the ring's runs leave as DMA blocks and `dma_isr()` (the
  line's ISR body) releases each and starts the next - a block start
  masks the engine's claim alone (six instructions, about 10 cycles,
  counted in the listing) and programs the channel with the mask down,
  two stores to the DMA block a block; with a receive
  engine the run is filled straight from UARTDR, a run that fills is
  published and re-armed by its completion on the line - `dma_isr()`
  answering the ring's edge - and a run still filling by `harvest()`,
  which publishes what TRANS_COUNT says has landed and re-arms; the
  error interrupts count each received error (the engine moves an
  entry's byte, a break's zero included, and not its flags), and no
  vector ends a burst under an engine
  ([../pl011/README.md](../pl011/README.md), "The receive side");
  `dma_faults()` counts the blocks abandoned after a bus error.

HOW THE TRANSMITTER STARTS, because the interrupt is an edge that
stays latched (the IP stratum's page, [../pl011/README.md](../pl011/README.md),
where the rule and its measurement on the RP2350's same block are):
a byte with nothing queued ahead of it and room in the FIFO is
written straight into UARTDR by `write_byte` (a run at a time by
`write_bulk`), so an idle transmitter takes 32 bytes with no
interrupt; a byte behind a full FIFO is queued in the ring and arms
the transmit interrupt, whose handler refills the FIFO once per fall
through its level and disarms it when the ring runs dry; a byte the
full ring refuses writes nothing. The handler is the ring's one
consumer, and the loop writes the data register only while the ring
is empty.

## How to use it

```cpp
constexpr brio::UartPins console_pins{
    .tx = {0, brio::PinFunction::uart},
    .rx = {1, brio::PinFunction::uart},
};
using Serial = brio::Uart<0, console_pins>;
constexpr Serial serial;

extern "C" void isr_uart0() { (void)Serial::isr(); }

int main() {
    SysClock::init();
    Serial::init(clock, 115200);
    brio::enable_interrupts();
    brio::print(serial, "hello", brio::crlf);
}
```

With the kernel, the edge feeds `SerialPort`:

```cpp
extern "C" void isr_uart0() {
    if (Serial::isr()) { brio::post<SerialLines>(brio::RxActivity{}); }
}
```

## Bench findings

- The divisor from `Clock::pclk_hz` at 125 MHz for 115200 is 67 +
  52/64, the datasheet's own worked example (115207 baud, 0.006 %),
  read back in UARTIBRD/UARTFBRD.
- The reference suite, green on both boards, on the second instance:
  - every frame format - 5..8 bits, none/even/odd, 1 or 2 stops -
    byte-exact on the loop-back at 115200;
  - the ladder from the floor to the ceiling at clk_peri 125 MHz:
    120 baud (65104 + 11/64), 300, 9600, 115200, 921600, 2 M, 4 M and
    7812500 (1 + 0/64), each byte-exact and taking its frames' time;
    119 and 7.9 M refused, and a rate within half a sixty-fourth of
    a divisor rounds onto it (7812501 is 7812500);
  - THE TWO DELIVERIES: a lone byte reaches the ring 371 us after
    its write at 115200 - one frame plus the 32-bit receive timeout;
    a burst of sixteen is delivered by the level, its first byte
    only when the sixteenth has landed (1401 us), which is why a run
    shorter than a level costs the timeout at the end (267 ms at 120
    baud);
  - a 1 ms break counts one BE (and one FE, on the same entry) and
    delivers no byte; 48 frames into the 32-deep FIFO with the line
    masked deliver exactly 32 in order and count one OE - from
    UARTRSR, as the entries never carry it;
  - 4096 bytes round the loop at 3 Mbaud byte-exact in 13.7 ms (the
    wire's own 13.65).
- ON THE CROSS LINK (this board's GP4 to the other's GP5 and back):
  512 bytes to a peer's echo and back byte-exact in 46 ms with no
  error counted; listening at 7E1 to the peer's 64 frames at 8N1,
  the eighth data bit lands where the parity bit is expected - 30
  parity errors counted and dropped, 34 frames accepted, no frame
  error - the attribution per entry, on a real wire.
- THE ENGINES (`test_rp2040_dma`): 4096 bytes through the loop-back
  at 3 Mbaud with an engine in each slot, byte-exact, 22 interrupts;
  the two traps of a receive engine - a break taken at enable over a
  low pad, and the request credits that outlive a completed run - are
  the transport's business now ([dma.md](dma.md)).
- THROUGH THE DEBUG PROBE'S BRIDGE, the console's own ladder fed by
  the host: byte-exact at 115200, 460800, 921600, 1 M, 2 M and 3 M
  baud (142 KB at 3 M with no error and no overrun, host to board);
  the bridge sends 8N1 whatever parity the host asks for, so a
  parity-error leg through it is a measure of the probe, not of the
  UART ([../probes/raspberry-pi-debug-probe.md](../probes/raspberry-pi-debug-probe.md)).
- The transport works on the wire in both directions through the
  Debug Probe's bridge at 115200: the pin-check tool's banner and its
  echo-with-code of every character typed, the kernel console's
  banner, replies and prompt through `SerialPort` and `print`, with
  the frame, parity, break and hardware-overrun counters and the ring
  overrun all at zero. The receive timeout interrupt is what
  delivers a single typed character - and it is also what exposed
  the crt's handler-name trap ([platform.md](platform.md)) before the
  first byte.

- THE TRANSMIT POLICY MEASURED ON THE WEACT RP2040 as on the RP2350's
  same PL011 r1p5 ([../rp2350/uart.md](../rp2350/uart.md)): the direct
  write into an idle FIFO, the transmit interrupt armed only behind a
  full FIFO - `test_rp2040_serial` 32 pass whole, a 4096-byte print 501
  interrupts at x 0.99 of the wire and 256 bytes 31 (`bench_rp2040`
  letter p).

- THE RECEIVE SIDE AND ITS EDGE (`bench_rp2040` letter u, UART1 under
  LBE at 125 MHz, the meter's stamps inside `isr`): 256 bytes through the
  interrupt receiver in 16 level entries and the time-out's, 13408
  cycles at 115200 and 13541 at 1 Mbaud - 53 a byte - where the level
  read with a flag test before every character took 16510 and 15500 (60
  a byte); the receive engine 390 cycles for the same 256, two
  completions. The edge from the burst's last stop bit (BUSY falling):
  the time-out's 3.2 frames at 115200, and at 1 Mbaud 1.6 to 2.7 frames
  from run to run (measured, not dissected); under the engine
  `UARTRIS.RTRIS` never rose (17 bytes and ten frames of silence: the
  FIFO empty, every byte moved), so the engine's burst edge is its
  owner's poll - 2.4 frames at 115200 to 43 at 1 Mbaud on a 1 ms tick.
- THE TRANSMIT SIDE through the same letter: 4096 bytes in 145 refills
  at 25 cycles a byte (27 before), wire-bound (x 1.00) to UARTCLK / 16 =
  7.8125 Mbaud; through the transmit engine in 40 to 47 blocks, about 3
  cycles a byte, wire-bound too.
- `tx_idle()` ON THE PAD (`test_rp2040_serial` letter t, eight frames of
  0xFF at 9600 on GP4 with the loop-back off, the pad read through SIO):
  true 0 us from the last stop bit's end, a bit being 104 us; under the
  transmit engine (`test_rp2040_dma` letter t) the same.
- THE COPY INTO THE RING: the byte loop 2 cycles and 9 a byte, the
  runtime's `memcpy` about 54 and under one a byte with the two ends
  aligned alike - the loop ahead at 8 bytes, `memcpy` at 12, the
  family's `copy_crossover`.
- AGAINST THE VENDOR: the pico-sdk 2.3.1 has no interrupt or DMA
  receive, only `uart_read_blocking` and `uart_write_blocking`, POLLED
  loops (a flag test and a data access a byte), run on the same board
  and loop in a scratch program: 58 cycles a byte reading characters
  already in the FIFO and 53 writing into an empty one, every cycle of
  the wire's time the CPU's. This transport's interrupt receiver costs
  53 a byte and its transmitter 25, and the core is free between
  entries.

## Not covered yet

Driver gaps, each with its reason:

- Hardware flow control (CTS/RTS, 4.2.4): two more wires on the
  cross link (GP6/GP7 are UART1's CTS/RTS); the pads are table 279's
  next two of each group.
- IrDA (SIR, the low-power counter): an IrDA transceiver.
- The stick-parity bit (SPS): born with a first protocol that uses
  it (a nine-bit address mark).
- The FIFO trigger levels other than the task's half and eighth: a
  program with a reason to move them; the resource writes any pair.

Implemented but not bench-verified, each with what would measure it:

- The bridge's ceiling board to host (the suite measures host to
  board): a `source` leg of the same host letter.
- The receive timeout at the rates of the ladder other than 115200 and
  1 Mbaud (letter u above) and 120 baud (the suite's cost at the
  floor): a timed lone byte per rung.
- `rebase` and `set_baud` under a running port: the suite.
- Why a burst of 17 at 1 Mbaud is told 1.6 to 2.7 frames after its last
  stop bit, under the 32 bit periods of the time-out (letter u; the
  M33 and Hazard3 read 3.26 on the same source): the entries timed
  against the RX pad would place each one.
