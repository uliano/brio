# SERCOM - USART mode (SAM C21)

> **PROVISIONAL.** The asynchronous, internal-clock, 16x-arithmetic
> USART is implemented and bench-verified end to end - the console
> personality. The SERCOM's other three personalities (SPI host and
> client, I2C host and client) and the USART chapter's own long tail
> (fractional baud, synchronous mode, handshaking, LIN, IrDA,
> auto-baud, DMA) are declared, not built. The list is in "Not
> covered yet".

Documents of record: SAM C20/C21 data sheet DS60001479M - SERCOM
common ch. 30 (the baud generator, 30.6.2.3 table 30-2), USART
ch. 31 - and errata DS80000740S items 1.17.15 and 1.17.16, both
encoded in code (1.17.4 and 1.17.14 are named where their fields
live). Driver: `samc/sercom.hpp` (`Sercom<n>` resource +
`Uart<n, pads, rx, tx>` task). The family fixture is
`test/family_samc/sercom.cpp` plus two negatives under
`tools/check_samc.sh`.

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
- **`Uart<n, pads, rx_size, tx_size>`** - the task: two SPSC rings
  (lock-free at any size here - `atomic_width` 4 - so 64/256 are
  console-class defaults, not a ceiling), `init(clock, baud, format)`
  speaking hertz off the clock tag, `isr()` as the ONE handler body
  with the AVR's edge-return contract intact (true on the RX ring's
  empty-to-non-empty transition - the kernel wakeup), try-semantics
  `write_byte`, `read_byte`, error counters, `rebase(hz)` for the day
  a dynamic clock exists, `release()`. Init order is deliberate:
  clocks, reset, configure, enable, and only THEN the pads to the
  SERCOM - the transmitter idles high before the pad leaves PORT, so
  no glitch start bit reaches the wire. `ByteTransport` and
  `ClockUser` are static_asserted.

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

## Not covered yet

Driver gaps (not built):
- The SPI and I2C personalities - their own future drivers.
- Within USART: fractional and 3x-arithmetic baud, synchronous mode
  and XCK, RTS/CTS handshaking, RS-485/TE, LIN, IrDA, collision
  detection, auto-baud, start-of-frame/RXS wake, 9-bit data uses,
  DBGCTRL policy (1.17.4: DBGSTOP does not actually halt
  transmission - the field is exposed, the erratum named).
- DMA triggers - the next campaign's subject, with the TX and RX
  engines planned as options that compile to zero when absent.
- A per-package pad table (which pins reach which pads): the same
  device-table job the PORT driver leaves open.

Implemented but not bench-verified:
- Frame formats other than 8N1 (parity, 5..7/9 bits, two stop);
  `rebase()` (no dynamic clock exists on this target to drive it);
  instances other than SERCOM5; `release()`; operation on the E/G
  variants (compile-checked only).
