# USART - the serial ports (STM32F4)

Documents of record: RM0090 Rev 22 ch. 30 (RM0390 Rev 6 ch. 25 and
RM0383 Rev 4 ch. 19 are its twins: one register description, three
manuals), the datasheets' alternate-function tables for the pads, and
the errata's USART items - ES0206 2.11.x, ES0298 2.13.x, ES0287 2.10.x:
"idle frame is not detected if the receiver clock speed is deviated",
"break frame is transmitted regardless of CTS", "guard time not
respected when data are sent on TXE events", "RTS is active while RE
or UE = 0", and their siblings - none of which shapes the transport
(the console has no idle detection, no break, no smartcard and no
flow control), each of which the mode's future user meets. Driver:
`stm32f4/usart.hpp` (`Usart<n>` the resource, `Uart<n, pins, rx_size,
tx_size, TxEngine, RxEngine, opts>` the task, the vocabulary
`UartFormat`, `UartOptions`, `UartPins`, `MuteConfig`, `LinConfig`,
`IrdaConfig`, `SmartcardConfig`, `UsartSyncConfig`, `UsartFlag`, the
divisor arithmetic), over the reserve's instance facts
(`stm32f4/device_tables.hpp`); `stm32f4/dma_engine.hpp` holds the
empty engine tag. The family fixture is `test/family_stm32f4/usart.cpp`
with the negatives that refuse an absent instance, flow control on a
UART, an engine slot naming anything but the tag, and one pad twice.
Bench: the console apps and the platform suite on the three boards.

## What the silicon does

**The classic STM32 USART**: SR/DR/BRR/CR1/CR2/CR3/GTPR - the block the
CH32V00x stratum drives under WCH's names, and NOT the STM32G0's. The
flags are cleared by READ SEQUENCES and not by a clear register (30.6.1):
PE, FE, NE, ORE and IDLE by reading SR then DR; TC by reading SR then
writing DR; RXNE by reading DR; LBD and CTS by writing 0 to SR. The
handler is written around those sequences: it reads SR once, decides
from that copy, and lets the DR read do the clearing - which is also
why an overrun without a byte still reads DR.

**Ten instances at most, and the NAME says what an instance has.**
USART1, USART2 and USART6 on every part but the F410Tx (no USART6),
USART3 and UART4/5 from the F405 class, UART7/8 on the F42x/F43x, F413
and F469 classes, UART9/10 on the F413 alone. Table 148 (RM0090 30.5):
UART4, UART5, UART7, UART8 have no synchronous mode, no smartcard, no
hardware flow control - and 30.6.5 adds that the 0.5 and 1.5 stop bits
are not theirs. USART1, USART6 and the F413's UART9/10 sit on APB2,
every other instance on APB1: the divisor divides ITS bus's clock, and
on this family the two buses differ from HCLK at every rate above their
ceilings ([clock.md](clock.md)) - 45 vs 90 MHz at 180.

**The baud divisor** (30.3.4): USARTDIV = fCK / (8 x (2 - OVER8) x
baud), a 12-bit mantissa and a 4-bit fraction in BRR. At OVER8 = 0 the
register value is fCK / baud in sixteenths with no field arithmetic;
at OVER8 = 1 the fraction has three bits and bit 3 must stay clear
(30.6.3), for twice the reachable rate at half the receiver's
tolerance. Below 16 the generator has nothing to divide by.

**Every instance has a vector of its own** on this family - USART1_IRQn
and so on - no sharing to sort out in a handler.

**The frame**: M selects 8 or 9 bits with the parity bit counted in,
so seven data bits exist only with parity and nine only without; STOP
gives 1, 0.5, 2 and 1.5 stop bits, the halves the FULL instances'.

**The modes exclude each other** the way 30.3.8 .. 30.3.11 say: LIN
wants CLKEN, STOP, SCEN, HDSEL and IREN clear; IrDA wants one stop bit
and neither LIN nor half duplex nor smartcard; the smartcard wants the
clock and 1.5 stop bits, which its verb sets itself; half duplex joins
TX and RX inside the chip and leaves the RX pad alone.

## Types and verbs

- `UartFormat{bits (seven|eight|nine), parity (none|even|odd), stop
  (one|half|two|one_and_half)}`, `uart_format_valid`,
  `usart_cr1_format`, `usart_cr2_stop`, `uart_data_mask`; the divisor
  pair `usart_brr(hz, baud)` / `usart_brr_over8` (optional: none when
  unreachable), `usart_actual_baud` / `_over8`, `usart_min_hz` /
  `_over8`.
- `Usart<n>` (monostate, refused for an absent instance) - `number`,
  `on_apb2`, `is_full`, `irq`, `regs()`; `bus_clock(on)`/`bus_clock()`,
  `reset()`; `enable`, `enabled`, `transmitter`, `receiver`;
  `configure(format, brr)` (the frame into CR1/CR2, the divisor into
  BRR; refused for an invalid frame, a half stop on a UART, a divisor
  below 16), `stop_bits`, `set_brr`/`brr`, `oversampling8(on)` (UE
  clear only)/`oversampling8()`, `actual_baud(fck)` in the register's
  oversampling, `one_bit_sampling`; `mute_mode(MuteConfig)`, `mute()`
  (refused under an address-mark wake while RXNE stands), `unmute`,
  `muted`; `lin(LinConfig)`/`lin_off`/`lin_enabled`, `send_break`,
  `break_pending`; `half_duplex(on)`/`half_duplex()`;
  `irda(IrdaConfig)`/`irda_off`/`irda_enabled`;
  `smartcard(SmartcardConfig)`/`smartcard_off`/`smartcard_enabled` (a
  FULL instance's, false elsewhere); `synchronous(UsartSyncConfig)`
  (TE and RE clear, a FULL instance's)/`synchronous_off`/
  `synchronous_enabled`; `flow_control(rts, cts)` (a FULL instance's),
  `rts_enabled`, `cts_enabled`; `dma_transmit`, `dma_receive`; the
  interrupt enables `interrupts(mask, on)`, `rxne_interrupt`,
  `txe_interrupt` (get and set), `tc_interrupt`, `idle_interrupt`,
  `parity_interrupt`, `break_interrupt`, `cts_interrupt`,
  `error_interrupt`; `status`, `flag(mask)`, `clear_flags(mask)` (the
  rc_w0 four), `clear_by_read`; `tx_empty`, `tx_complete`, `rx_ready`,
  `write_data`/`write_word`, `read_data`/`read_word`, `data_address`.
  `UsartFlag` spells SR's bits, `receive_errors` and `rc_w0`.
- `UartPins{tx, rx}` as `PinSel`s, `UartOptions{over8, one_bit,
  half_duplex, rts, cts, rts_pin, cts_pin, tx_speed}` - the defaults
  are the console's.
- `Uart<n, pins, rx_size = 64, tx_size = 256, TxEngine = NoDmaEngine,
  RxEngine = NoDmaEngine, opts = {}>` - `init(clock, baud, format = 8N1)`
  (the divisor from `apb_hz(clock, on_apb2)`, false when unreachable or
  for nine data bits), `isr()` (the vector's body: the RX edge as its
  return), `write_byte`, `read_byte`, `write`, `write_bulk`,
  `read_bulk`, `rx_pending`, `tx_idle`, the counters `rx_overruns`,
  `frame_errors`, `parity_errors`, `noise_errors`, `hw_overruns`,
  `clear_errors`, `rebase(hz)` (the ClockUser verb: `hz` is SYSCLK and
  the bus rate is DERIVED from it with `apb_hz_at`, not read back from
  the RCC, because a dynamic clock fans the new rate out BEFORE the
  prescalers move), `set_baud(hz, baud)`, `divisor_for`,
  `min_hz_for`, `can_baud`, `actual_baud(fck)`, `kernel_hz<Clock>()`,
  `release()`. Refused at compile time: invalid or coincident pads, an
  engine slot naming anything but `NoDmaEngine`, flow control on a
  UART, a flow pad missing.

## How to use it

```cpp
constexpr brio::UartPins console_pins{
    .tx = {'A', 9, brio::PinFunction::af7},    // USART1_TX (the datasheet's AF table)
    .rx = {'A', 10, brio::PinFunction::af7},   // USART1_RX
};
using Serial = brio::Uart<1, console_pins>;
constexpr Serial serial;

extern "C" void USART1_IRQHandler() {
    if (Serial::isr()) { brio::post<SerialLines>(brio::RxActivity{}); }
}

Serial::init(clock, 115200);                       // false: the rate is unreachable
brio::print(serial, "hello", brio::crlf);
const uint32_t real = Serial::actual_baud(Serial::kernel_hz<SysClock>());
```

A faster link: `constexpr brio::UartOptions fast{.over8 = true, .tx_speed
= brio::PinSpeed::high};` then `brio::Uart<1, pins, 64, 256,
brio::NoDmaEngine, brio::NoDmaEngine, fast>`. A mode beyond the
transport - a LIN break, a muted receiver - through `brio::Usart<1>`'s
verbs on the same instance.

## Bench findings

- The console runs byte-exact on the three boards at 115200 8N1:
  USART2 on PCLK1 45 MHz with BRR 391 (115089 baud) on the
  Nucleo-F446RE, USART1 on PCLK2 90 MHz with BRR 781 (115236) on the
  DISC1, USART1 on PCLK2 100 MHz with BRR 868 (115207) on the black
  pill - the `ERR` verb reporting every counter zero after the
  platform suite's 36 verdicts and the console session.
- The error counters count: a host sweeping other baud rates against
  the Nucleo's console left 36 frame errors and 2 noise flags on the
  receiver, no hardware overrun, and the bytes DROPPED (the line
  assembler saw none of them).
- The TX ring drains through TXE and disarms when dry (CR1 read back
  0x202C - UE, TE, RE, RXNEIE, TXEIE clear - with the ring's indices
  equal after the banner); the RX path fills the ring and the edge
  posts (the ring's indices consumed by `SerialPort`). A DR poked by
  the debugger reached the host, which is how a stalled-looking console
  was told apart from a dead transmitter during the bring-up: the
  transmitter was fine and the host had sent `\r` where the line
  assembler wants `\n`.
- The black pill's RX pad, unconnected before the bridge's wires were
  crossed right, read idle under its pull-up: an empty RX ring and no
  framing noise, which is what pointed at the wiring.
- `rebase(hz)` carries the console across a whole rate ladder: six
  rates from 180 MHz down to the 16 MHz HSI and back, the bus divider
  changing from /4 to /1 under it, every line legible
  ([clock.md](clock.md), `test_stm32f4_power` letter f).
- AND `tx_idle()` IS NOT THE WIRE: it reports the transport's ring, and
  the last character is still in the shift register when it answers
  true. A Stop taken there truncates that character - measured, the
  line's own CRLF lost - so a program that stops its clocks waits for
  the USART's TC as well ([pwr.md](pwr.md)).

## Not covered yet

Driver gaps:
- The DMA engine slots hold the empty tag alone: this family's
  stream-and-FIFO controller (RM0090 ch. 10) is the DMA chapter's, and
  the engines come with it.
- Nine-bit words through the TASK (the resource's `write_word`/
  `read_word` speak them; the rings carry bytes) - declined, as on the
  other strata.
- A wake from Stop: this USART has no wake unit (the G0's WUF); the
  power chapter says what a Stop does to a pending receive.

Implemented, not bench-verified (every mode of the resource beyond the
console's - each waits for the wire or the peer that measures it):
OVER8 and ONEBIT (a peer at a rate the console does not use), mute
mode with both wakes, LIN's break and detection, single-wire half
duplex (a second board on the wire), IrDA (an IR pair), the smartcard
(no card on the desk), the synchronous clock (a timer capture on CK -
the timer chapter's ruler), CTS/RTS flow control (a peer that asserts
them), the DMAT/DMAR bits (the DMA chapter), the IDLE/TC/PE/CTS/LBD
interrupt enables, `set_baud` at run time (nothing changes the LINK's
rate with the clock standing still), `write_bulk`/`read_bulk` (compiled, the
console writes bytes), `release()`, the instances beyond the consoles'
(USART3, USART6, the UARTs - compiled on every header that has them,
none driven), the frame formats beyond 8N1 (parity and two stops
compile; a peer measures them).
