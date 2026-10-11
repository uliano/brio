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
  than 3%" (14.3). A divisor of 16 carries bytes on a wire (measured,
  below).
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
  LBD, with its own interrupt enable. Both are measured, the detection
  on a wire (below).
- **Single-wire half duplex** (14.5, HDSEL): TX alone, the pad "in
  output mode plus pull" - an alternate PUSH-PULL output with an
  external pull, the only alternate output this series has - with LIN,
  the smartcard, IrDA and the clock off. 14.5's "the TX and RX pins
  connected internally on the chip" is the receive path from the TX
  pad: it hears ANOTHER node driving that pad while the pad is an input,
  floating or pulled up, and it does not hear the instance's own frames
  with the pad driven (both measured, below) - so a bus of two nodes on
  this series turns each node's pad between the function and an input,
  and the die has no jumperless self-loop.
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
true on the edge from an empty receive ring), `write_byte` and
`write_bulk` (a run: as many as fit, its first byte pushed and TXEIE
armed before the rest is copied, armed again behind it), `read_byte` and
`read_span`/`consume` (the receive run in place), `tx_idle()`,
`rx_pending()`, `rx_skips()` (the gaps in the stream the receive ring
hands out, never cleared - util/serial_port.hpp's epoch: the SkipRing's
skips, one at the consumer's look after `isr()` dropped a flagged byte
or a full ring refused one); the counters
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

## Bench findings

The reference suite is `test_x035_usart` on a CH32X035F8U6 - WCH's
evaluation board in its QFN20 edition, over a WCH-LinkE - at 48 MHz, the
console on USART2 and the instance under test USART4 on its default
column, PB0 and PB1, which are the desk's link: crossed to a CH32V006K8U6
(its USART2 on column 4) running `usart_peer` on its 24 MHz crystal
doubled. `z` (23 verdicts) is the single-board set, run with the far
board dark - on its console or with its peer still searching, both
leaving their link pads floating inputs; `y` (66 verdicts) is the
two-board set against that peer, every leg judged from what the far chip
reports. What it measured:

- **The divisor counts HCLK** (letter `a`): the console's BRR is 417 at
  48 MHz - 26 and 1/16 - which `actual_baud()` puts at 115107 baud, 0.08
  per cent under 115200; 3 Mbaud is a divisor of exactly 16, and 3.2
  Mbaud has none. Down the clock suite's ladder the same port's divisor
  is HCLK / baud at every rung, from 417 at 48 MHz to 26 at 3 MHz, and
  the console reads clean at each ([clock.md](clock.md)).
- **Every frame lands as the chapter spells it** (letter `b`), register
  by register on USART4 with the port disabled and BRR holding 417
  through each:

  | frame | CTLR1 | CTLR2 |
  |---|---|---|
  | 8N1 | 0x0000 | 0x0000 |
  | 8E1 | 0x1400: M and PCE, the parity bit the word's ninth | 0x0000 |
  | 7O2 | 0x0600: PCE and PS, a word of eight | 0x2000: STOP = 10 |
  | 9N1.5 | 0x1000: M | 0x3000: STOP = 11 |

  Seven data bits with no parity bit are refused, and so is a divisor
  below 16, BRR keeping 417.
- **The modes' bits and their exclusions** (letter `c`), each written,
  read back and refused where 14.4 to 14.7 say: LIN takes LINEN and the
  11-bit detection, with half duplex and IrDA refused under it; half
  duplex takes HDSEL, with LIN and the smartcard refused under it; IrDA
  refuses two stop bits, takes the low-power mode with a prescaler of 7
  and holds the stop bits at one while it is on; the smartcard takes a
  guard time of 12, a prescaler of 5, 1.5 stop bits, CLKEN and NACK, and
  `smartcard_off()` leaves CLKEN to `synchronous_off()`; the synchronous
  clock is refused while TE is set and takes CPOL, CPHA and LBCL with the
  port disabled; the flow-control pair takes RTSE and CTSE; mute mode
  takes the address-mark wake, the address 9 and RWU.
- **A frame's length** (letter `d`): one 8N1 byte of the console timed
  from the DATAR write to TC took 4400 HCLK cycles, against 4170 for ten
  bit times of the divisor - 10.55 bit times.
- **THE RATE ASKED, on the transmitter's own clock** (letter `d`, a RUN
  polled one frame ahead of the shifter, from the first store to TC on
  the ticker's cycle count, against its wire time at HCLK over the baud
  asked): the console's 128 frames at 115200 in 1001 thousandths (BRR 417
  for 416.7 is 0.8 slow), and USART4 on PB0 1024 frames in 1000 at
  115200 and at 1 Mbaud and 1001 at 3 Mbaud, a divisor of exactly 16 -
  with the receive pad PULLED UP: it is a wire to the far board's
  floating input, and with it floating the 3 Mbaud run took 2054
  thousandths, the receiver's invented frames taking the core.
- **The break** (letter `e`): SBK set on USART4 at 115200 cleared itself
  4347 HCLK cycles later, 10.4 bit times of the 417-cycle divisor.
- **The debug port's column is refused** (letter `f`): with SW_CFG at 0,
  `init()` of USART3 on its code 1 - PC18 and PC19 - answers false,
  USART3's gate is as it was and its remap field at the reset code.
- **Half duplex does not hear its own frames** (letter `h`): HDSEL on
  PB0, the pad on the function, four bytes sent and TC reached for each,
  none raising RXNE.
- **THE HSI AGAINST A CRYSTAL** (letter `j`): the far chip timed a
  gapless run of 4000 frames at 1 Mbaud - a divisor of 48, so the HSI's
  error alone - first frame to last on its crystal's clock: -2480 to -2701 ppm at the
  factory calibration (HSICAL 0x53, the trim at 16) over two sessions'
  runs, the oscillator wandering by tens to a few hundred ppm between
  runs a fraction of a second apart (every later letter's reference is
  measured back to back with what it judges). THE TRIM stepped from 7 to
  28 and each step timed the same way: -25610 ppm at 7, -292 at 17 (the
  trim nearest 48 MHz on this die), +25302 at 28; every step up, between
  71 and 183 kHz, the mean 116 to 118 kHz against the manual's "about
  110KHz" (3.4.1), and back at 16 within one step of the first reading.
- **The rates on the wire, timed by the foreign receiver** (letter `k`):
  at 115200, 921600, 1, 1.5, 2, 2.4 and 3 Mbaud sixteen bytes echoed
  equal and counted clean by the far chip, and 2000 gapless frames timed
  within 500 ppm of HSI / BRR - the 1 Mbaud reference taken just before
  each - where a wrong divisor anywhere on that ladder is 2400 ppm off or
  more: 115200 at -3778 ppm against -3746 (the divisor's -0.08 per cent
  on the HSI's), 921600 at -1436 against -1276 (the divisor's +0.16), 3
  Mbaud at -2830 against -2901.
- **The transport across the wire** (letter `k`): eight bytes through
  `Uart` echoed by the far chip come back equal at 115200 and 1 Mbaud;
  64 bytes it sends at 3 Mbaud are all counted by the far chip; a gapless
  run of 256 INTO it, taken by runs (`read_span()`/`consume()`), arrives
  whole and in order at 1, 1.5 and 2 Mbaud. THE CEILING, measured: at
  2.4 Mbaud the receive ring fills behind the main loop (31 bytes
  refused, the run broken at byte 128) and at 3 Mbaud the shift register
  overruns (ORE 15, the run broken at byte 21); a full-duplex echo at 3
  Mbaud loses bytes too. The transport takes one vector entry a frame,
  the hardware prologue's round trip of 100 to 104 cycles
  ([platform.md](platform.md)) and the receive body of about 30
  instructions in the release listing beside it, against a frame of 160
  cycles at 3 Mbaud and 200 at 2.4 - the engines' work, not written in
  this stratum (below).
- **The frame matrix across the wire** (letter `l`): 8N1, 8E1, 8O1, 7E1,
  7O1, 9N1, 8N2, 8N0.5 and 8N1.5 at 115200, eight frames each echoed
  equal and counted clean; HALF A STOP BIT and one and a half timed as
  frames of 19 and 21 half bits at 1 Mbaud (-2857 and -2836 ppm against
  the 8N1 reference's figure, a frame half a bit off being 5 per cent).
- **The receiver's errors, from a crystal sender** (letter `m`): six
  odd-parity frames into an even receiver raise PE six times and FE
  never; 0x00 frames from the far chip's edge engine at 9600 off by up to
  plus and minus 5 per cent are clean, with this receiver's own -0.26
  per cent folded in - inside 14.3's "not less than 3%" with margin - FE
  rising from -6 per cent (every frame at -6 and -8); a fast sender's
  0x00 frame raises no FE up to +8 per cent, its stop sample landing in
  the idle that follows. Low pulses on the idle line at 9600 (a bit of
  2500 ticks of 24 MHz): 6 to 1250 ticks are no start bit, 1875 is one
  with NE, a whole bit is one. A run of eight read late: ORE with RXNE,
  DATAR keeping the FIRST frame (0xB0) and the seven after it lost. A
  break of 20 bit times: one frame, FE, data 0x00.
- **The far chip's engine shapes** (letter `n`): six bit-banged 0x55
  frames clean; one cell (data bit 5 of 0x00) stretched survives to +40
  per cent and fails from +50 - the stop sample's half bit; a 13-bit
  break ahead of a frame raises LBD in LIN mode and the frame after it
  arrives.
- **The modes on a wire** (letter `o`): muted under the address-mark
  wake with ADD 5, 9-bit words from three groups addressed 3, 5 and 3 -
  the receiver delivers the address frame 0x105 and its three data and
  nothing else; muted after the first frame of three runs of four 3 ms
  apart under the idle-line wake, it drops that run's other three and
  wakes on the idle line for the next two (nine frames, 0 and 4..11);
  IrDA's SIR frames at 115200, normal mode, echoed equal by the far
  chip's codec; LIN's detection: a 13-bit break raises LBD at both
  lengths, a 9-bit low at neither, a 10-bit low not at 11.
- **Half duplex hears a second chip** (letter `p`): HDSEL on PB0 with the
  pad an input - floating, and pulled up - receives the four 0x5A frames
  the far chip drives onto that wire from its own RX pad.
- **A pad handed to the function as TE rises shows a low pulse** (the
  far chip's USART, measured across this wire, the same block's
  lineage): the link's end parks its TX pad as a GPIO output high across
  a reconfiguration and hands it back two bit times and 20 us after the
  enable.

## Not covered yet

Driver gaps, each with its reason:

- **The DMA engine slots**: the DMA chapter is not written in this
  stratum ([README.md](README.md)), so `TxEngine` and `RxEngine` take
  `NoDmaEngine` alone; the resource's `dma_transmit`/`dma_receive` and
  the channel numbers are there for the engine that fills them. Above 2
  Mbaud sustained reception is theirs (the transport's measured ceiling,
  above).
- **The synchronous, smartcard, IrDA, mute and flow-control modes as
  tasks** (the AVR's `SyncHost`, `IrdaLink` and the like): the resource's
  verbs are the whole of it, and a task is born with its first user.

Implemented but not bench-verified, each with what would measure it:

- **What the other modes do on a wire**: the smartcard's guard time and
  NACK, the clock on CK and the SENSE OF LBCL want a logic analyser on
  the pads or a peer on a CK pad this package does not bond, and the
  flow-control pair's handshake a peer on the CTS and RTS pads (PB15 and
  PA8, unbonded on the QFN20).
- **IrDA's low-power mode across the wire**: letter `o` echoes the
  normal mode; the low-power pulse wants the far chip's decoder at the
  same prescaled width, a second leg of that letter.
- **USART1 and USART3, and the other packages' columns**: USART1 has no
  usable column on the QFN20 and USART3's only one is the debug port's,
  so neither has carried a byte here; a board of another package would
  measure them and the columns this one does not bring out.
