# UART (RP2040)

Documents of record: the RP2040 datasheet (build 3184e62), 4.2
(UART: the ARM PL011 - 4.2.2 functional description, 4.2.3 operation,
4.2.6 interrupts, 4.2.7 the programmer's model and the baud
calculation of 4.2.7.1, 4.2.8 the registers), 2.19.2 (table 279: the
pins), 2.15.3.1 (clk_peri). The driver: `brio/rp2040/uart.hpp`
(`Pl011<n>` the resource, `Uart<n, pins, ...>` the task) over
`pin.hpp`, `resets.hpp`, `clock.hpp` and `util/ring.hpp`. The
reference suite: `test_rp2040_serial` (the second instance under the
loop-back for the formats, the ladder, the FIFOs, a break, an overrun
and bulk traffic; a peer board on the cross link; the host at the
console's other end).

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
its trigger level - a transition, not a level, so enabling it over
an empty FIFO raises nothing (4.2.6.3) - and the four errors. The
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
  ICR), `UartDataError` (UARTDR's flags), `UartFifoLevel` (the
  trigger levels in eighths).
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
  `write_byte`, `read_byte`, `write`, `write_bulk`, `read_bulk`,
  `rx_pending`, `tx_idle`, `rebase(hz)`, `set_baud(hz, baud)`,
  `set_format(format)`, `loopback(on)` (each under the running port,
  after a drain), `min_hz_for`, `can_baud`, `actual_baud(hz)`,
  `release()`; the counters `rx_overruns` (ring), `frame_errors`,
  `parity_errors`, `break_errors` (a break entry carries FE too, and
  both are counted), `hw_overruns` (one per overrun event, from
  UARTRSR), `clear_errors`. The engine slots take `NoDmaEngine`
  until `dma.hpp` exists.

HOW THE TRANSMITTER STARTS, because the interrupt is a transition:
`write_byte` queues the byte and PENDS THE INSTANCE'S LINE in the
NVIC; the handler moves the ring into the FIFO until the FIFO is
full or the ring is empty, and arms the transmit interrupt only when
bytes remain queued - the FIFO draining through its level is then a
real transition. The handler is the ring's one consumer, the loop
never writes the data register.

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
    wire's own 13.65) with 12 to 13 interrupts per hundred bytes: the
    FIFO levels at half and an eighth batch the work.
- ON THE CROSS LINK (this board's GP4 to the other's GP5 and back):
  512 bytes to a peer's echo and back byte-exact in 46 ms with no
  error counted; listening at 7E1 to the peer's 64 frames at 8N1,
  the eighth data bit lands where the parity bit is expected - 30
  parity errors counted and dropped, 34 frames accepted, no frame
  error - the attribution per entry, on a real wire.
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
  overrun all at zero. The pended-line transmit start (the file
  header) is what carried every byte of it; the receive timeout
  interrupt is what delivers a single typed character - and it is
  also what exposed the crt's handler-name trap
  ([platform.md](platform.md)) before the first byte.

## Not covered yet

Driver gaps, each with its reason:

- Hardware flow control (CTS/RTS, 4.2.4): two more wires on the
  cross link (GP6/GP7 are UART1's CTS/RTS); the pads are table 279's
  next two of each group.
- IrDA (SIR, the low-power counter): an IrDA transceiver.
- The stick-parity bit (SPS): born with a first protocol that uses
  it (a nine-bit address mark).
- The DMA engines (UARTDMACR, the request lines): with `dma.hpp`; the
  slots are shaped and empty.
- The FIFO trigger levels other than the task's half and eighth: a
  program with a reason to move them; the resource writes any pair.

Implemented but not bench-verified, each with what would measure it:

- The bridge's ceiling board to host (the suite measures host to
  board): a `source` leg of the same host letter.
- The receive timeout at the other rates of the ladder: the suite
  measures it at 115200 and sees its cost at 120; a timed lone byte
  per rung.
- `rebase` and `set_baud` under a running port: the suite.
