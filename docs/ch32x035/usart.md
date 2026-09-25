# USART (CH32X035)

The serial ports of the CH32X035 series, in the two strata every brio
serial driver has ([../design/serial.md](../design/serial.md)):
`Usart<n>`, the RESOURCE - which instance, its gate, its vector, its DMA
channels and chapter 14 as verbs - and `Uart<n, P, ...>`, the TASK - the
interrupt-driven byte transport every console runs on, with the sibling
strata's surface and template parameters.

Documents of record: the CH32X035 reference manual V1.8 (14.1 for what
the block has, 14.3 for the divisor, 14.4 to 14.7 for the synchronous,
half-duplex, smartcard and IrDA modes and what each excludes, 14.8 and
14.9 for the DMA and the interrupts, 14.10 for the registers; 8.3.2.1
for the remap columns, 9.2.3 for the DMA requests, 3.4.6 and 3.4.7 for
the gates, table 7-1 for the vectors) and the CH32X035/X033 datasheet
V1.7 (the model table's count of serial ports per part, table 2-1 for
the pads each package bonds). The header is
[brio/ch32x035/usart.hpp](../../brio/ch32x035/usart.hpp), with the
columns in [afio.hpp](../../brio/ch32x035/afio.hpp) and the request
channels in [dma_engine.hpp](../../brio/ch32x035/dma_engine.hpp); the
reference suite is `test_x035_usart`.

## What the silicon does

### Four instances, one clock

- **USART1 answers on PB2 and USART2, USART3 and USART4 on PB1**
  (3.4.6, 3.4.7), and every one of them counts its divisor in **HCLK**
  (14.3): the series has no peripheral-bus prescaler, so a port's rate
  is the clock's and nothing else. The vectors are 32, 39, 42 and 43
  (table 7-1).
- **Every instance is a full USART**: the chapter counts four USARTs and
  gives each the synchronous clock, LIN, IrDA, the smartcard, DMA and a
  fractional divider "up to 3Mbps" (14.1), and each register set carries
  multiprocessor mute and the flow-control pair (14.10.4 to 14.10.6).
  There is no UART here, so no verb is a compile error on one instance
  and not another.
- **Which instances a part offers is read off its PINS, not its
  count.** A column is a transport's where its TX pad may be driven
  (bonded and alone on its package pin) and its RX pad is bonded, and an
  instance is offered where some column is:

  | part | offered | the model table's count |
  |---|---|---|
  | CH32X035R8T6, C8T6, G8U6, G8R6 | USART1, USART2, USART3, USART4 | 4 |
  | CH32X035F8U6 | USART2, USART3, USART4 | 3 |
  | CH32X035F7P6 | USART2, USART3 | 3 |
  | CH32X033F8P6 | USART1, USART2, USART3, USART4 | 4 |

  The CH32X035F8U6 has no USART1 column with both pads - PB10 and PB11
  are USART1's default and the QFN20 bonds PB11 alone, and the other
  three columns miss a pad each - so it has no USART1. The
  CH32X035F7P6's count of three is not reachable: its only USART4
  columns with both pads bonded put TX on PC16 or PC17, which share
  their pins with PC11 and PC10 on that package, and table 2-1's note 4
  forbids both as outputs. The part table follows the note, and keeps
  the count beside it as the datasheet gives it.
- **On the 20-pin parts USART3 lives on the debug port.** Its only
  bonded column there is code 1, TX on PC18 and RX on PC19 - SWDIO and
  SWCLK - so the instance exists and a transport on it is refused while
  the probe's port is alive ([pin.md](pin.md)).

### The frame and the divisor

- **BRR is the divisor in sixteenths**, a 12-bit mantissa over a 4-bit
  fraction: the rate is HCLK / (16 x USARTDIV) (14.3, 14.10.3), so the
  value to store is simply HCLK / baud, rounded - 417 for 115200 at 48
  MHz, which gives 115108 baud, and exactly 16 for 3 Mbaud. Below 16
  there is nothing to divide by. The receiver's tolerance is "not less
  than 3%" (14.3).
- **The word is M: eight or nine bits, the parity bit included**
  (14.10.4), so seven data bits exist only with parity and nine only
  without; PCE and PS choose none, even or odd; STOP gives 1, 0.5, 2 or
  1.5 stop bits (14.10.5).
- **Errors are read, then cleared.** PE, FE, NE, ORE and IDLE stand in
  STATR and go when STATR is read and then DATAR; RXNE is cleared by the
  DATAR read and TXE by the DATAR write; RXNE, TC, LBD and CTS also
  clear by a zero written over them (14.10.1).

### The modes

- **Mute mode** (14.10.4, 14.10.5): RWU puts the receiver to sleep until
  an idle line or a frame whose MSB is set and whose low four bits are
  this node's ADD; under the address-mark wake RWU cannot be written
  while RXNE stands.
- **LIN** (LINEN): SBK sends a break, which the hardware clears on the
  break's stop bit; a break of 10 or 11 bits (LBDL) is detected into
  LBD, with its own interrupt enable.
- **Single-wire half duplex** (14.5, HDSEL): TX alone, the pad "in
  output mode plus pull" - an alternate PUSH-PULL output with an
  external pull, the only alternate output this series has - with LIN,
  the smartcard, IrDA and the clock off.
- **IrDA** (14.7, IREN and IRLP): the SIR encoder and decoder, normal
  mode at the bit rate or the low-power mode on the prescaled clock
  (GPR.PSC, eight bits; 1 in normal mode, 0 a hold); LIN, STOP, the
  clock, the smartcard and half duplex off.
- **The smartcard** (14.6, SCEN): a NACK on a parity error, the guard
  time in bit times (GPR.GT), the card's clock on CK through the low
  five bits of GPR.PSC; LIN, half duplex and IrDA off, the clock kept.
- **The synchronous clock** (14.4, CLKEN): CK driven for each data bit,
  CPOL and CPHA, and LBCL - which THIS MANUAL describes as "1: the clock
  pulse for the last bit of data is not output" (14.10.5), where WCH's
  own library names the same bit `USART_LastBit_Enable`. CPOL, CPHA and
  LBCL cannot change once the transmitter is enabled; the smartcard,
  half duplex and IrDA off.
- **Flow control** (14.10.6): RTSE and CTSE, CTS with its flag and its
  interrupt.
- **DMA** (14.8, 9.2.3): DMAT and DMAR, each direction on a channel of
  its own - USART1 on channels 4 (TX) and 5 (RX), USART2 on 7 and 6,
  USART3 on 2 and 3, USART4 on 1 and 8.

## Types and verbs

The vocabulary, shared with the sibling strata: `UartFormat` (`bits`
seven/eight/nine, `parity` none/even/odd, `stop` one/half/two/
one_and_half; 8N1 by default) with `uart_format_valid`, `MuteConfig`,
`LinConfig`, `IrdaConfig`, `UsartSyncConfig` (`clock_idle_high`,
`capture_second_edge`, `last_bit_clock` - named for the BIT as the
register spells it, not for a promise), `SmartcardConfig`, and the
arithmetic `usart_divisor(hclk, baud)`, `usart_divisor_valid`,
`usart_actual_baud`, `usart_min_hz`. Per instance: `usart_bus_for`,
`usart_gate_for`, `usart_irq_for`, `usart_column_for(n, code)` (the
five pads, afio.hpp's data), `usart_pads_for(n, code)` (TX and RX),
`usart_remap_valid(n, code)` (a column a transport may use on this
part) and `usart_column_on_debug_port(n, code)`.

`Usart<n>`, the resource, refuses at compile time an instance number
outside 1..4 and one the part does not offer. It carries `bus`, `pads`,
`irq`, `dma_tx_channel`, `dma_rx_channel` and `is_full` (true on every
instance), and the verbs by purpose - each stores what it names and
nothing else, and a verb whose combination the chapter declares
undefined answers false and writes nothing:

| purpose | verbs |
|---|---|
| the block | `bus_clock(on)`, `reset()`, `remap(code)` |
| the frame | `configure(format, brr)` (with the port disabled, the registers written whole), `stop_bits(s)`, `set_brr(v)`, `brr()`, `enable(on)`, `enabled()`, `transmitter(on)`, `receiver(on)`, `actual_baud(hclk)` |
| mute | `mute_mode(config)`, `mute()`, `unmute()`, `muted()` |
| LIN | `lin(config)`, `lin_off()`, `lin_enabled()`, `send_break()`, `break_pending()` |
| half duplex | `half_duplex(on)`, `half_duplex()` |
| IrDA | `irda(config)`, `irda_off()`, `irda_enabled()`, `prescaler()` |
| the smartcard | `smartcard(config)`, `smartcard_off()`, `smartcard_enabled()`, `guard_time()` |
| the synchronous clock | `synchronous(config)`, `synchronous_off()`, `synchronous_enabled()` |
| flow control | `flow_control(rts, cts)`, `rts_enabled()`, `cts_enabled()` |
| DMA | `dma_transmit(on)`, `dma_receive(on)`, `data_address()` |
| interrupts | `rxne_interrupt`, `txe_interrupt`, `tc_interrupt`, `idle_interrupt`, `parity_interrupt`, `break_interrupt`, `cts_interrupt`, `error_interrupt`, and `interrupts(mask, on)` for CTLR1's |
| flags and data | `status()`, `flag(mask)`, `clear_flags(mask)` (the write-zero ones), `clear_by_read()`, `tx_empty()`, `tx_complete()`, `rx_ready()`, `read_word()`, `write_word(w)` (nine bits), `read_data()`, `write_data(b)`, `take_errors()` |

`Uart<n, P, rx_size, tx_size, format, TxEngine, RxEngine, remap, opts>`,
the transport - the CH32V203's parameter list, `UartOptions` its last
parameter (`half_duplex`, `rts`, `cts`). It refuses at compile time an
instance the part does not offer, a nine-bit or an invalid format (the
rings carry bytes; nine bits are the resource's `read_word`), a column
that is not a transport's on this part, an RTS or CTS pad the column's
package does not give, and an engine other than `NoDmaEngine`.
`init(clock, baud)` answers false when the divisor comes out below 16,
and when the column puts TX or RX on the debug port's pads while the
probe's port is alive; otherwise it opens the gate, writes the column
if it is not the reset one, hands TX to the peripheral before the enable
and leaves RX a floating input, configures the frame and the divisor,
and arms RXNE and the vector. Then: `isr()` (the vector's whole body,
true on the edge from an empty receive ring), `write_byte`, `read_byte`,
`write(buffer, len)`, `tx_idle()`, `rx_pending()`; the counters
`rx_overruns`, `hw_overruns`, `frame_errors`, `noise_errors`,
`parity_errors` (saturating) and `clear_errors()`; `baud()`,
`actual_baud(hclk)`, `divisor_for`, `min_hz_for`, `can_baud`;
`rebase(hz)` - the port as a `ClockUser`, draining what it holds at the
old rate before writing the new divisor - `set_baud(hz, baud)` for the
link's own rate, and `release()`. `dma_isr()`, `harvest()` and
`dma_faults()` are there for the surface and answer false and zero.

## How to use it

The console, on the QFN20's USART2 - its default column, PA2 and PA3:

```cpp
using P = brio::Ch32x035Platform<>;
using Serial = brio::Uart<2, P>;          // rings of 64 and 64, 8N1
constexpr Serial serial;                  // the tag print() takes

extern "C" BRIO_CH32_INTERRUPT void usart2_handler() {
    if (Serial::isr()) {
        brio::post<SerialLines>(brio::RxActivity{});
    }
}

Serial::init(clock, 115200);
brio::print(serial, "hello", brio::crlf);
```

Another column and another frame - USART4 on PB0/PB1, even parity:

```cpp
using Link = brio::Uart<4, P, 64, 64, brio::UartFormat{brio::UartBits::eight, brio::UartParity::even}>;
```

A column that is not the reset one, and the flow-control pair (on a
package that bonds both pads):

```cpp
using Port = brio::Uart<2, P, 64, 64, brio::UartFormat{}, brio::NoDmaEngine, brio::NoDmaEngine,
                        0, brio::UartOptions{.rts = true, .cts = true}>;
```

The chapter beyond the transport, on the resource:

```cpp
brio::Usart<4>::lin({.break_11bit = false, .break_interrupt = true});
brio::Usart<4>::send_break();
```

## Not covered yet

Driver gaps, each with its reason:

- **The DMA engine slots**: the DMA chapter is not written in this
  stratum ([README.md](README.md)), so `TxEngine` and `RxEngine` take
  `NoDmaEngine` alone; the resource's `dma_transmit`/`dma_receive` and
  the channel numbers are there for the engine that fills them.
- **The synchronous, smartcard, IrDA, mute and flow-control modes as
  tasks** (the AVR's `SyncHost`, `IrdaLink` and the like): the resource's
  verbs are the whole of it, and a task is born with its first user.

Implemented but not bench-verified, each with what would measure it:

- **The divisor**: the console's BRR for 115200 at 48 MHz, the rate it
  gives, and the arithmetic at 3 Mbaud: `test_x035_usart` letter a.
- **The frame, register by register** on USART4 with the port disabled
  (8N1, 8E1, 7O1, 9N1, the four stop codes) and the refusals: letter b.
- **The modes and their exclusions**, register by register - LIN, half
  duplex, IrDA, the smartcard and the synchronous clock each refused
  where 14.4 to 14.7 say and read back where not: letter c. What the
  modes do on a wire - the IrDA pulse, the smartcard's guard time and
  NACK, the clock on CK and the SENSE OF LBCL - wants a logic analyser
  on the pads, which no letter has.
- **A frame's length on the wire**: one console byte timed from the
  DATAR write to TC, against ten bit times of the divisor: letter d.
- **The break**: SBK set on USART4 and the time until the hardware
  clears it: letter e.
- **The debug port's refusal**: `init()` of USART3's column 1 answering
  false and touching nothing: letter f.
- **Bytes across a wire**: USART4 talking to itself through a jumper
  PB0-PB1 at 115200, 1 Mbaud and 3 Mbaud through the transport, 8E1, 7O1
  and 9N1 polled on the resource, and a LIN break detected: letter `w`,
  which detects the jumper before it judges.
- **The HSI's accuracy as a UART's**: a console that reads clean at
  every rung of the clock ladder is the proof that HCLK lands within the
  receiver's tolerance ([clock.md](clock.md), `test_x035_clock` letter
  b).
- **USART1 and the other packages' columns**: USART1 has no usable
  column on the QFN20; a board of another package would measure it and the
  columns this one does not bring out.
