# USART (STM32G0)

> **PROVISIONAL.** The asynchronous, oversampling-by-16, PCLK-clocked
> USART with 7/8/9-bit words, parity and 1/2 stop bits, interrupt
> driven in the non-FIFO register view, is implemented and
> bench-verified end to end - the console personality. The chapter's
> long tail (the FIFOs, the prescaler, the other kernel clocks and the
> wake-up from Stop they enable, synchronous mode, flow control,
> single-wire, LIN, IrDA, smartcard, Modbus, auto-baud, the receiver
> time-out, DMA, the LPUARTs) stays declared, not built. The list is
> in "Not covered yet".

Documents of record: RM0444 Rev 6 - USART ch. 33 (the implementation
tables 183/184, the baud generator 33.5.7, the FIFOs 33.5.4, the
registers 33.8) - and errata ES0548 Rev 3 items 2.11.1 (stated: the
noise counter exists for it) and 2.11.2 (applied: the prescaler is
not written). Driver: `stm32g0/usart.hpp` (`Usart<n>` resource +
`Uart<n, pins, rx, tx>` task); the instance, vector and bus-clock
facts come from `stm32g0/device_tables.hpp`. The family fixture is
`test/family_stm32g0/usart.cpp` plus three negatives under
`tools/check_stm32g0.sh`.

## What the silicon does

**Six instances on the G0B1, four on the G071, two on the G031 - and
they are not copies of each other.** RM0444 table 183: USART1 is FULL
everywhere; USART2 is FULL on the G071 class and up, BASIC on the
G031; USART3 FULL on the G0B1, BASIC on the G071; USART4..6 BASIC.
FULL brings the 8-deep TX/RX FIFOs, the PRESC prescaler, synchronous
mode, smartcard/IrDA/LIN, auto-baud, the receiver time-out and the
Stop wake-up (table 184). Which instance is which is a POINTER
COMPARISON macro in the device header (`IS_UART_FIFO_INSTANCE`), not
a constexpr fact, which is one reason this stratum drives the
non-FIFO view every instance has. The instances' bus clocks differ
too (USART1 on APB2, the rest on APB1), as do their kernel-clock
multiplexers (USART1 everywhere, USART2 from the G071 class,
USART3 on the G0B1 class; the rest run on PCLK, full stop) and their
VECTORS (USART2 shares with LPUART2 on the G0B1, alone elsewhere;
USART3..6 share one line with LPUART1) - every one of these a probe
in the reserve, proven on all three headers by the fixture.

**Configuration is written with UE clear.** BRR, CR1's frame fields,
CR2, CR3 and PRESC are all "can only be written when the USART is
disabled": the task disables around every configuration, including a
rebase.

**The baud generator is a plain integer divisor** (33.5.7, OVER8 = 0):
BRR = USARTDIV = usart_ker_ck / baud, USARTDIV >= 16 - the simplest of
brio's three targets (the SAM scales a fraction into 65536, the AVR
into 64). Rounded to nearest and reported back through `actual_baud`:
556 at 64 MHz for 115200 is 115107 baud, -0.08 %.

**TXE is a condition and ORE is a storm.** TXE reads 1 whenever the
data register is empty, so its interrupt is armed only while the ring
holds something and disarmed from the handler when it runs dry. ORE
raises the interrupt whenever RXNEIE is set (33.8.9) and is cleared
ONLY through ICR.ORECF: a handler that reads RDR and leaves ORE
standing re-enters for ever - the SERCOM ERROR storm the samc bench
caught, in this family's clothes; the handler here clears every error
it counts. RDR holds the last GOOD byte when ORE is set; FE/PE/NE in
the non-FIFO view belong to the byte in RDR, so a framed or
parity-failed byte is dropped precisely, a noisy one is kept and
counted (ES0548 2.11.1 is a noise story).

**TE sends an idle frame first** (33.5.5), which is why the pads are
handed to the peripheral BEFORE UE/TE are raised - and why the RX pad
gets a pull-up, so an unconnected line reads idle instead of noise.

## Types and verbs

- `Usart<n>` - `regs()`, `irq()`, `bus_clock(on)`, `kernel_clock
  (UsartClock)` (true only for what the instance can do: PCLK on an
  instance with no multiplexer), `configure(UartFormat, brr)` (refused
  while enabled), `enable`/`enabled`, `brr`, `status`, `clear_flags`,
  `read_data`/`write_data`, `rxne_interrupt`/`txe_interrupt` (under
  the guard: CR1 is shared with the handler), `reset()` (through
  RCC's APB reset register).
- `Uart<n, pins, rx_size = 64, tx_size = 256>` - `init(clock, baud,
  format)` (false when the rate is unreachable at PCLK), `isr()` (the
  ONE body; true on the RX ring's empty-to-non-empty edge), `write_byte`
  / `read_byte` (the `ByteTransport` pair print() and SerialPort use),
  `write`, `write_bulk`/`read_bulk` (the ring's span API, one nudge),
  `rx_pending`, `tx_idle`, `rebase(hz)` (ClockUser), `actual_baud`,
  `can_baud`, `min_hz_for`, the counters `rx_overruns` (ring full),
  `hw_overruns` (ORE), `frame_errors`, `parity_errors`, `noise_errors`,
  `clear_errors`, `release()`.
- `UartPins {tx, rx}` of `PinSel` (port, pin, AF), `uart_pins_valid`;
  `UartFormat {bits, parity, stop_bits}` (8N1 default), `UartBits`,
  `UartParity`; `UsartFlag`; `UsartClock` (pclk/sysclk/hsi16/lse);
  `usart_brr`, `usart_actual_baud`, `usart_min_hz` - constexpr,
  fixture-pinned with the chapter's own two examples.

## How to use it

```cpp
constexpr brio::UartPins console_pins{
    .tx = {'A', 2, brio::PinFunction::af1},   // USART2_TX, DS13560 table 13
    .rx = {'A', 3, brio::PinFunction::af1},   // USART2_RX
};
using Serial = brio::Uart<2, console_pins>;
constexpr Serial serial;

extern "C" void USART2_LPUART2_IRQHandler() {     // the SHARED line's name
    if (Serial::isr()) { brio::post<SerialLines>(brio::RxActivity{}); }
}

Serial::init(clock, 115200);                      // after SysClock::init()
brio::print(serial, "hello", brio::crlf);
```

A second instance at 8E2, with its own rings:

```cpp
using Aux = brio::Uart<1, aux_pins, 128, 512>;
Aux::init(clock, 9600, {.parity = brio::UartParity::even, .stop_bits = 2});
```

## Bench findings

USART2 through the Nucleo-G0B1RE's ST-LINK virtual COM port, the
console app, 115200 8N1 at PCLK = 64 MHz. BRR 556 read back over SWD
(CR1 0x2D = UE + RE + TE + RXNEIE, TEACK and REACK set, the NVIC's
bit 28); `actual_baud` 115107. Three hundred 5-byte command lines
answered by three hundred 49-byte replies, byte-exact, with every
counter at zero - rx ring, hardware overrun, frame, parity, noise. A
280-byte burst fired with no pacing into the 64-byte RX ring (forty
lines whose replies each block the loop for ~6 ms) lost three lines,
and said so: 21 ring overruns counted, hardware overruns 0 - the loss
is in the software ring by design, accounted and never silent. The
banner arrives on a reset issued over SWD. The kernel tick against
the host over ten seconds: +0.24 %.

Two things the bring-up learned about the tools: `tools/bench.py run`
sends the bench suites' single-letter commands with no line
terminator, so it does not drive this line-oriented console (any
serial monitor does); and SWD memory reads through the ST-LINK's HLA
transport are unreliable while the core sleeps in WFI (README.md).

## Not covered yet

Driver gaps:
- FIFO mode (FIFOEN, the thresholds, the FIFO-view flag names) and the
  PRESC prescaler - the FULL instances' features, per instance.
- Oversampling by 8, the fractional half of the baud arithmetic
  (BRR[3:0] under OVER8), the receiver tolerance table.
- Kernel clocks other than PCLK (HSI16, SYSCLK, LSE) and the wake-up
  from Stop they make possible; HSIKERON.
- Synchronous mode, hardware flow control (RTS/CTS, driver enable),
  single-wire half-duplex, LIN, IrDA, smartcard, Modbus, auto-baud,
  the receiver time-out, character match, the swap/invert options,
  the break request, DMA, the LPUARTs (their own baud arithmetic:
  256 x the clock).
- A pin-table check of the AF claim (the header has no such table).

Implemented, not bench-verified: 7- and 9-bit words, parity, 2 stop
bits, `rebase`, `release`, `write_bulk`/`read_bulk`, `Usart::reset`,
any instance but USART2, the frame/parity/noise counters ever
counting (every run so far was clean).
