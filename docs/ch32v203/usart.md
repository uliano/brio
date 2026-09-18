# USART (CH32V203)

The serial ports of RM chapter 18 - the asynchronous frame in every
shape the register description allows, the fractional baud generator,
mute mode with both wakes, LIN's break, single-wire half duplex, IrDA,
the smartcard, the synchronous clock, the flow-control pair, two DMA
requests, every flag and both interrupt sources - and the
interrupt-driven byte transport every brio console is, sitting on that
resource with its options costing nothing. Documents of record: the
CH32FV2x_V3xRM reference manual V2.3 (18.2 for the frame and TE's idle
frame, 18.3 for the baud generator, 18.4 for the synchronous mode, 18.5
for half duplex, 18.6 for the smartcard, 18.7 for IrDA, 18.8 for DMA,
18.9 for the interrupts, 18.10 for the registers; 3.4 for the gates and
the two peripheral buses; 11.2.3's table 11-5 for the DMA channels) with
its CLASS RULE - ours is CH32V20x_D6 - the CH32V203 datasheet V2.8
(table 2-1 for how many serial ports a part has, the pin tables of 3.2
for which), AFIO's tables 10-23 to 10-27 (the columns,
[pin.md](pin.md)), the driver
[brio/ch32v203/usart.hpp](../../brio/ch32v203/usart.hpp) and the
reference suite `test_v203_serial`.

## What the silicon does

- **Four instances on two buses, and which of them a PART has is the
  datasheet's table.** USART1 answers on PB2 and counts PCLK2; USART2,
  USART3 and the fourth port answer on PB1 and count PCLK1, which this
  stratum caps at 72 MHz ([clock.md](clock.md)) - so at a system clock
  of 144 MHz the two buses divide different numbers and every divisor
  is asked of the instance's own. The packages below the CH32V203C8
  count two serial ports and the smallest counts ONE, and it is not
  USART1: `device::has_usart(n)` is a mask, not a count.
- **THE FOURTH PORT IS A USART ON THIS PART.** Chapter 18's opening
  counts three USARTs and five UARTs for the whole family and then names
  the exception - "for CH32V203C8 the serial port 4 is a universal
  synchronous asynchronous receiver transmitter" - and the datasheet's
  pin table agrees, bringing out USART4_CK, USART4_CTS and USART4_RTS.
  On the CH32V203RB, the other device class, it is a UART4 with TX and
  RX alone. So the synchronous clock, the smartcard and the
  flow-control pair are a PART fact (`device::usart_full`), and the
  verbs behind them refuse rather than write bits an instance has not
  got. Measured: the fourth port's CK pad carries a clock, and its
  vector is the one at this class's own tail.
- **The pads are AFIO's columns** (tables 10-23 to 10-27), which the
  driver takes from the same table that programs the register - TX, RX,
  CK, CTS and RTS move together. UART4 reads a DIFFERENT TABLE per
  class: the column that starts at PC10/PC11 is the bigger class's,
  and the CH32V203C8 is named in the other, whose default pads are PB0
  and PB1 and whose second column is PA5/PB5/PA6/PA7/PA15. USART2's
  second column (PD5/PD6) belongs to the other class and is refused
  here; a column with no pad bonded on the package is refused too.
- **The bit IS the divisor.** BRR counts peripheral clocks per
  sixteenth of a bit, so the whole register is pclk/baud and a start bit
  lasts exactly BRR peripheral clocks: measured to the clock at 9600,
  115200, 921600 and 3 Mbaud. Every standard rate from 1200 to 3 Mbaud
  comes out within ONE per mille of what was asked on both buses.
- **The divisor's range refuses at the SLOW end, not the fast one.**
  Sixteen is the floor (one whole clock per bit sixteenth) and 65535 the
  register: from a 144 MHz bus, 1200 baud asks for 120000 and is refused,
  while every rate up to 3 Mbaud is far above the floor. `init()`
  answers false rather than mangling every frame.
- **The frame is a word length plus a parity choice**: M gives 8 or 9
  bits INCLUDING the parity bit, so seven data bits exist only with
  parity and nine only without. Measured on the wire as the low run of a
  zero word: nine bit times for 8N1 and 7E1, ten for 8E1 and for a
  nine-bit word.
- **Two of the four stop codes are an ordinary frame's.** The gap
  between two zero words sent back to back is one bit time under the 1
  code and two under the 2 code - and ALSO one and two under the 0.5 and
  1.5 codes (measured): an ordinary transmitter rounds the halves up.
  18.10.5 offers all four and the halves belong to the smartcard, whose
  own verb sets 1.5 for the card.
- **TE sends an idle frame first** (18.2): the first byte written after
  the enable waits a whole frame for TXE - 1146 us at 9600, where a
  frame is 1042 us. **TXE is the shift register taking the byte, not the
  frame leaving**: written to a busy transmitter it comes back 1041 us
  later, written to an idle one 104 us later, and TC a whole frame after
  that.
- **The errors are read then cleared**: PE, FE, NE, ORE and IDLE by
  reading STATR and then DATAR, in that order (18.10.1); RXNE, TC, LBD
  and CTS also by writing a zero over them. A frame whose parity does not
  add up raises PE and IS STILL DELIVERED - the receiver reports, it does
  not drop - and a low stop bit raises FE. On an overrun the DATA
  REGISTER's byte survives and the shift register's is lost (measured:
  the first of two frames is what comes out), and the read sequence
  clears ORE.
- **IDLE rises once** after a frame followed by a line that stays high,
  and not again until RXNE has been set (18.10.1's note, measured).
- **Mute mode**: with RWU set, frames are not received; under WAKE 0 an
  idle line clears RWU by itself and the next frame is taken; under
  WAKE 1 a nine-bit frame whose MSB is set and whose low four bits are
  ADD wakes the receiver, and another node's address does not.
  18.10.4's two notes are real: a byte must have been received before an
  idle-line wake works, and RWU cannot be set under an address mark
  while RXNE stands (`mute()` refuses).
- **The break** SBK sends is TEN bits of low outside LIN mode and
  THIRTEEN inside it (measured on the transmit pad: 150000 and 195000
  timer clocks at 9600, where a bit is 15000). The detector raises LBD
  once at ten low bits and not at nine; with LBDL set it wants eleven and
  ten are a frame; LBDIE carries it to the vector.
- **IrDA's two directions do not share a level logic** (18.7 says so,
  and here is what it means): the ENCODER puts a HIGH pulse of three
  sixteenths of a bit on an idle-LOW pad for every zero (measured 2814
  timer clocks for the 2812 of 3/16 at 9600), or three periods of the
  prescaled clock in low-power mode (1200 for 1200 with a prescaler of
  200); the DECODER takes the MIRROR - an idle-HIGH line whose zeros are
  LOW pulses. A frame banged the encoder's way reads 0xFF, the same
  frame banged the other way reads its byte.
- **THE SYNCHRONOUS MODE IS THE MASTER'S ALONE** (18.4): the column's CK
  pad carries one pulse per data bit while the TRANSMITTER shifts and at
  no other time, and CPOL is the pad's idle level (measured both ways).
  **LBCL reads as the F1 family's and not as 18.10.5's words**: the
  manual says the pulse of the last data bit is output when the bit is
  CLEAR, and the silicon does the opposite - eight bytes give 112
  transitions (seven pulses each) with LBCL clear and 128 (eight) with
  it set. CPHA is the RECEIVER's capture edge and moves nothing on the
  pad (the same eight bytes, the same 112 transitions). The three bits
  take a write only with TE and RE clear, and the mode excludes LIN,
  half duplex, IrDA and the smartcard.
- **The clock does not fill the receiver.** 18.4 says the receiver "will
  only sample when outputting the clock"; what it still wants is a start
  bit of its own, so a clocked frame with an idle receive line raises no
  RXNE at all (measured: status 0xC0, the pad high).
- **The smartcard's card clock runs free, but only with TE enabled.**
  Its own verb writes SCEN, the NACK, the guard time, the prescaler, 1.5
  stop bits and CLKEN - the clock is the card's and belongs to the mode.
  On the pad it is the peripheral clock divided by TWICE the prescaler
  (measured 6001500 Hz for the 6 MHz a prescaler of 6 asks of a 72 MHz
  bus) and it keeps running while nothing is being sent; with the
  transmitter disabled the pad stays still. The prescaler is the low
  five bits: one to thirty-one, and zero is reserved.
- **Half duplex needs a pull-up the pads cannot give.** With HDSEL the
  port drives the one wire only while it transmits and releases it
  otherwise - that is what a bus is - and this family's pads carry no
  pull in any output mode (10.2.7), so on a board with no resistor the
  line is held by nothing: the frame's low run measures its nine bit
  times in one run and a wandering number in the next, and the byte the
  instance's own receiver reads back changes with it. The mode's own
  bits are what this board can judge; the wire is what it cannot.
- **The flow-control pair** is the receiver's and the transmitter's own
  hands: CTS sampled high holds the next frame in the data register - TC
  never comes - and a low level lets it out at once; RTS is low while the
  receiver can take a frame, high while a received byte waits unread and
  low again when it is read. A change on the CTS line raises its flag in
  both directions and CTSIE carries it to the vector.
- **The DMA requests are channels** (table 11-5, and the channel IS the
  request on this family - [dma.md](dma.md)): USART1 transmits on 4 and
  receives on 5, USART2 on 7 and 6, USART3 on 2 and 3, the fourth port
  on 1 and 8. An engine on any other channel is refused at compile time.
- **What the register file has not got**: CTLR4 with its MARK and SPACE
  parity, CTLR1's M_EXT (five, six and seven-bit words) and STATR's
  MS_ERR and RX_BUSY all carry the same note naming the CH32F20x_D8, the
  CH32V30x and the CH32V31x. The register view stops at GPR and no verb
  reaches past it.

## Types and verbs

[brio/ch32v203/usart.hpp](../../brio/ch32v203/usart.hpp), two strata:

- The vocabulary: `UartBits` (seven, eight, nine DATA bits),
  `UartParity`, `UartStop` (the four codes), `UartFormat` and
  `uart_format_valid()`, `usart_ctlr1_format()` / `usart_ctlr2_stop()`
  (the register fields of a format), `uart_data_mask()`, `MuteWake` +
  `MuteConfig` (+ `mute_valid()`), `LinConfig`, `IrdaConfig` (+
  `irda_valid()`), `UsartSyncConfig`, `SmartcardConfig` (+
  `smartcard_valid()`), `usart_divisor(pclk, baud)` with
  `usart_divisor_valid()`, `usart_actual_baud()` and `usart_min_hz()`;
  the instance facts `usart_base_for()`, `usart_bus_for()`,
  `usart_gate_for()`, `usart_irq_for()`, `usart_remap_of()`,
  `usart_remap_valid()`, `usart_column_for()` / `usart_pads_for()` (the
  five pads of a column, and the two a transport claims),
  `usart_dma_tx_channel()` / `usart_dma_rx_channel()`.
- `Usart<n>` is the resource: `bus_clock()`, `reset()`, `remap(code)`,
  `enable()`/`enabled()`, `transmitter()`, `receiver()`,
  `configure(UartFormat, brr)` (the frame and the divisor written whole
  with the port disabled, refusing a format the word length cannot carry
  or a divisor out of range), `stop_bits()` (refused under IrDA),
  `set_brr()`/`brr()`/`actual_baud()`, `mute_mode()`/`mute()`/
  `unmute()`/`muted()`, `lin()`/`lin_off()`/`lin_enabled()`,
  `send_break()`/`break_pending()`, `half_duplex()` both ways,
  `irda()`/`irda_off()`/`irda_enabled()`, `prescaler()`/`guard_time()`,
  `smartcard()`/`smartcard_off()`/`smartcard_enabled()`,
  `synchronous()`/`synchronous_off()`/`synchronous_enabled()`,
  `flow_control(rts, cts)` + `rts_enabled()`/`cts_enabled()`,
  `dma_transmit()`/`dma_receive()`/`data_address()`, the interrupt
  enables (`interrupts(mask)`, `rxne_interrupt()`, `txe_interrupt()`,
  `tc_interrupt()`, `idle_interrupt()`, `parity_interrupt()`,
  `break_interrupt()`, `cts_interrupt()`, `error_interrupt()`),
  `status()`/`flag()`/`clear_flags()` (the write-zero flags alone)/
  `clear_by_read()`/`take_errors()`, `tx_empty()`/`tx_complete()`/
  `rx_ready()`, `write_data()`/`write_word()`/`read_data()`/
  `read_word()`; and the constants `number`, `bus`, `irq`, `pads`,
  `dma_tx_channel`, `dma_rx_channel`, `is_full`. Every mode refuses the
  company the chapter forbids it, and a verb that is a full USART's
  answers false on an instance that is not one.
- `Uart<n, P, rx_size, tx_size, format, TxEngine, RxEngine, remap,
  opts>` is the transport task, on that resource: `init(clock, baud)`,
  `isr()` (the edge), `write_byte()`/`read_byte()`/`write()`,
  `rx_pending()`/`tx_idle()`, the counters (`rx_overruns()`,
  `frame_errors()`, `parity_errors()`, `noise_errors()`,
  `hw_overruns()`, `clear_errors()`), `baud()`/`actual_baud()`/
  `set_baud()`/`rebase()`/`divisor_for()`/`can_baud()`/`min_hz_for()`,
  `release()`, and the engine verbs `dma_isr()`, `harvest()`,
  `dma_faults()`. THE FRAME IS ITS OWN PARAMETER here, ahead of the
  engine slots, and `UartOptions` - the trailing one - carries what is
  left: `half_duplex` (the TX pad as an alternate-function open drain,
  the RX pad untouched) and `rts`/`cts` (the column's pads, refused on
  an instance that is not full and on a package that does not bond
  them). A remap code of 0 writes no AFIO register at all.

## How to use it

The console, as every app has it:

```cpp
using Serial = brio::Uart<1, P>;                 // USART1, PA9/PA10, 8N1
constexpr Serial serial;
Serial::init(clock, 115200);
extern "C" BRIO_CH32_INTERRUPT void usart1_handler() { (void)Serial::isr(); }
```

A second port with a frame of its own and the flow-control pair, on a
column:

```cpp
constexpr brio::UartOptions link_opts{.rts = true, .cts = true};
using Link = brio::Uart<4, P, 64, 64, brio::UartFormat{.parity = brio::UartParity::even},
                        brio::NoDmaEngine, brio::NoDmaEngine, 1, link_opts>;
Link::init(clock, 9600);                          // TX PA5, RX PB5, CTS PA7, RTS PA15
```

The transport with both DMA engines, on the channels this instance's
requests are wired to:

```cpp
using Loop = brio::Uart<2, P, 256, 256, brio::UartFormat{},
                        brio::DmaTxEngine<7>, brio::DmaRxEngine<6>>;
extern "C" BRIO_CH32_INTERRUPT void dma1_channel7_handler() { (void)Loop::dma_isr(); }
extern "C" BRIO_CH32_INTERRUPT void dma1_channel6_handler() { (void)Loop::dma_isr(); }
// ... and harvest() whenever the owner wants to publish what arrived
```

The chapter beyond the transport, on the resource - a LIN break, a
muted receiver waiting for its address, and the synchronous clock:

```cpp
using U = brio::Usart<2>;
U::lin({.break_11bit = true, .break_interrupt = true});
U::send_break();                     // thirteen bits of low, then the flag on the next one seen

U::mute_mode({.wake = brio::MuteWake::address_mark, .address = 5});
U::mute();                           // asleep until a 9-bit frame carries 0x105

using Clocked = brio::Usart<4>;      // a full USART on this part
Clocked::transmitter(false);
Clocked::receiver(false);            // CPOL, CPHA and LBCL want both clear
Clocked::synchronous({.clock_idle_high = false, .last_bit_clock = true});
// the CK pad is the caller's: brio::usart_column_for(4, 1).ck is PA6 here
```

The smartcard, whose verb takes the card's clock with it:

```cpp
U4::smartcard({.nack = true, .guard_time = 16, .clock_prescaler = 6});
// SCEN, NACK, GT, PSC, 1.5 stop bits and CLKEN; PCLK1 / 12 on the CK pad
U4::smartcard_off();                 // SCEN and its NACK; the clock is synchronous_off()'s
```

## Bench findings

The reference suite is `test_v203_serial` (44 verdicts in `z`) on the
CH32V203C8T6 at 144 MHz - PCLK2 144, PCLK1 72 - with USART2 as the
instrument on its default pads and the console untouched on USART1.
Almost every letter needs no wire, and three facts of this silicon are
why: the receive pad is a bit-banged transmitter (an input follows its
own pull, which the output data register moves in a few cycles), a pad
driven by a peripheral still reaches a TIMER CAPTURE input on the same
pad (USART2's TX is TIM2's channel 3, the fourth port's remapped CK is
TIM3's channel 1), and the fourth serial port is a USART whose clock pad
can be counted. Every fact above with a number is this suite's
measurement. Two letters want a strap and detect its absence: the
loopback PA2 to PA3, and the crossed pair PA2-PB1 with PB0-PA3 - the
second is measured below.

- **The baud table**, printed by the suite: twenty-eight rows, every
  standard rate from 1200 to 3 Mbaud against both peripheral clocks,
  with the divisor, the rate it really gives and the error in per mille.
  Nothing exceeds one per mille; the one refusal is 1200 baud from the
  144 MHz bus.
- **The ruler**: TIM2's channels 3 and 4 both on PA2 - one direct and
  one through the cross-mapping this family's timers have, since there
  is no both-edge capture here - so the transmit pad's levels are
  measured with nothing strapped. A capture register holds the LAST
  event of its polarity, which is why the runs that matter are read edge
  by edge as they arrive.
- **A stale CTS enable spins the vector.** With CTSIE armed and the CTS
  pad moved by software the handler was re-entered without end, although
  it cleared the flag on every pass; the suite's body drops the enable
  as it does for TXE and each change is armed on its own. Whether the
  flag's source stands or the line bounced under its own pull is not
  separable from this measurement.
- **The crossed pair**, USART2's TX strapped to the fourth port's RX and
  the fourth port's TX to USART2's RX: the two instances talk to each
  other in both directions at 8N1 and 115200 baud and at 8E2 and 19200,
  sixty-four xorshift bytes each way with every byte arriving in order
  and no error flag on either side.

## Not covered yet

Driver gaps, each with its reason:

- **A single-wire bus with a peer**, and with the pull-up the chapter
  asks for: the mode is armed and its exclusions hold, but the wire is
  released between frames and nothing on this board holds it - a
  resistor to the supply on the TX pad is what would make the frame's
  timing and the receiver's own echo measurable.
- **The smartcard's data path**: the register face, the guard time, the
  NACK bit and the card clock are measured, but ISO 7816-3 itself - the
  answer to reset, the NACK a parity error provokes, the guard time
  delaying TC - wants a card and a socket on the desk.
- **The synchronous mode against a peer**: this side is the master and
  its CK pad is counted, so what a slave puts on RX in step with that
  clock is a second board's answer.
- **USART3**: the instance is on the part, and its default column is
  PB10 and PB11 - the same two pads I2C2 answers on
  ([i2c.md](i2c.md)). Nothing on this board speaks to a third serial
  port; it arrives with the first program that needs one.
- **CTLR4, M_EXT, MS_ERR and RX_BUSY**: another device class's, by their
  own notes; nothing here reaches them.

Implemented but not bench-verified, each with what would measure it:

- **The two DMA engine slots on this chapter**: the request table is
  checked at compile time, the DMA document measures the channels
  themselves and the two bus chapters measure the same engine types on
  a wire ([dma.md](dma.md), [spi.md](spi.md)), but a round trip
  through THIS transport - the transmit engine draining the ring and
  the receive engine filling it, published by `harvest()` - is the
  loopback letter's, and PA2 to PA3 is the one of the two straps above
  that the board does not carry.
- **Four kilobytes at 921600 baud**: the loopback letter's, waiting on
  the same strap.
- **`error_interrupt()` (EIE, the FE/ORE/NE vector under DMAR)**: an
  error provoked on a line a receive engine is draining is what would
  raise it.
- **The transport's `remap` parameter on a column other than the
  default**: the resource's `remap()` is measured through the fourth
  port's second column, and the transport's own path is compiled and
  not run.
- **`set_baud()`, `release()` and `rx_pending()`**: the surface the
  other five strata share is compiled here and exercised by the family
  fixture; the loopback letter is where `set_baud()` would be measured
  on the wire.
