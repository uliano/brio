# USART - the serial port (AVR DA/DB)

> **PROVISIONAL.** Not the result of a systematic review of the USART
> chapter and errata; the driver covers the asynchronous 8N1 byte
> transport every app's console uses. The exhaustive pass is pending.
> Documents consulted: AVR128DB28/32/48/64 data sheet DS40002247B
> (USART, electricals 39.14), errata DS80000915F (2.16.x: open-drain
> with TXD output, start-of-frame detection in active mode, receiver
> dead after ISFIF in auto-baud modes - none used). Driver:
> `avrdx/uart.hpp`; the line service above it: [serial.md](../design/serial.md)
> (`util/serial_port.hpp`). Reference tests: `console`, `clock_console`
> (rebase under a running console), every app's console.

## What the driver does today

`Uart<n, Route, rx_size, tx_size>`: asynchronous, 8 data bits, no
parity, 1 stop bit, interrupt driven on both sides with SPSC rings
(defaults 64/256, 8-bit indices: lock-free on AVR). TX is "try"
semantics (`write_byte` false when the ring is full; `print` spins on
it - bounded by the wire rate); RX bytes go into the ring from the
RXC body, which returns the empty -> non-empty edge the app glue
posts as an event. Error counters (frame, parity, buffer overflow, ring
overrun) are named statics. Baud from `clock_hz(clock)` (normal-speed
formula, BAUD >= 64 i.e. CLK_PER >= 16 x baud); a `ClockUser`: `rebase`
drains TX and reloads BAUD. Routes: default or ALT1 per USART
(PORTMUX); the bench console is USART2 ALT1 on PF4/PF5.

## Types and verbs

| Entity | Verbs |
|--------|-------|
| `Uart<n, Route::def/alt1, rx_size, tx_size>` | `init(clock, baud)`, `rebase(hz)`, `can_baud(hz, baud)`, `min_hz_for(baud)` |
| bytes | `write_byte(b)` -> bool, `write(buf, len)` -> written, `read_byte(b&)` -> bool, `rx_pending()`, `tx_idle()` |
| ISR bodies | `rxc()` -> bool (edge), `dre()`; the app binds `USARTn_RXC_vect` / `USARTn_DRE_vect` |
| errors | `rx_overruns()`, `frame_errors()`, `parity_errors()`, `hw_overruns()`, `clear_errors()` |
| as a ByteSink/ByteSource | `print(serial, ...)` works on the tag object |

## How to use it

```cpp
using Serial = brio::Uart<2, brio::Route::alt1>;
constexpr Serial serial;
ISR(USART2_RXC_vect) { if (Serial::rxc()) brio::post<Lines>(brio::RxActivity{}); }
ISR(USART2_DRE_vect) { Serial::dre(); }
...
Serial::init(clock, 460800);
brio::print(serial, "hello ", 42, brio::crlf);
uint8_t b; if (Serial::read_byte(b)) { ... }
```
Lines and commands: `SerialPort` ([serial.md](../design/serial.md)).

## Bench findings

- `write_byte` ~45-50 cycles/byte; the lock-free ring path is 13
  instructions; TX worst-case stall ~2 ms at 460800 (ring full).
- Rebase under a running console verified 24 -> 12 -> 2 MHz at 115200.

## Not covered yet

Synchronous mode (XCK), host SPI mode, one-wire and RS-485 half
duplex, 5/6/7/9 data bits, parity, two stop bits, double-speed
(CLK2X), multiprocessor mode, start-of-frame detection (wake from
standby), IrDA, LIN, auto-baud, the TXC interrupt, open-drain TXD,
errata 2.16.x behaviours in those modes, the fractional baud
generator's accuracy table.
