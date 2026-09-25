# USART (CH32V203, CH32V303)

The serial ports of RM chapter 18 - the asynchronous frame in every
shape the register description allows, the fractional baud generator,
mute mode with both wakes, LIN's break, single-wire half duplex, IrDA,
the smartcard, the synchronous clock, the flow-control pair, two DMA
requests, every flag and both interrupt sources, and on the CH32V303 the
MARK and SPACE parity and the short words of a LOT - and the
interrupt-driven byte transport every brio console is, sitting on that
resource with its options costing nothing. Documents of record: the
CH32FV2x_V3xRM reference manual V2.3 (the chapter's opening for which
ports are USARTs, 18.2 for the frame and TE's idle frame, 18.3 for the
baud generator, 18.4 for the synchronous mode, 18.5 for half duplex,
18.6 for the smartcard, 18.7 for IrDA, 18.8 for DMA, 18.9 for the
interrupts, 18.10 for the registers with 18.10.1, 18.10.4 and 18.10.8's
notes for the lot's four features; 3.4.7 and 3.4.8 for the gates and the
two peripheral buses; 11.2.3's tables 11-2 to 11-5 for the DMA slots;
9.5.1 and table 9-2 for the vectors) with its CLASS RULE - three classes
read the chapter here: CH32V20x_D6 for every CH32V203 up to the
CH32V203C8, CH32V20x_D8 for the CH32V203RB and CH32V30x_D8 for the four
CH32V303 - the CH32V203 datasheet V2.8 (table 2-1 for how many serial
ports a part has, the pin tables of 3.2 for which) and the CH32V303
datasheet V3.5 (table 2-1-1, the pin tables of 3.2 and the alternate
functions of 3.3), AFIO's tables 10-23 to 10-31 (the columns,
[pin.md](pin.md)), the driver
[brio/ch32vx03/usart.hpp](../../brio/ch32vx03/usart.hpp) and the
reference suite `test_vx03_serial`.

## What the silicon does

### Up to eight instances on two buses

- **Which ports a PART has is the datasheet's table.** USART1 answers on
  PB2 and counts PCLK2; every other port answers on PB1 and counts
  PCLK1, which this stratum caps at 72 MHz ([clock.md](clock.md)) - so at
  a system clock of 144 MHz the two buses divide different numbers and
  every divisor is asked of the instance's own. The packages below the
  CH32V203C8 count two serial ports and the smallest counts ONE, and it
  is not USART1; the CH32V203C8 and RB count four; the 128 KB CH32V303
  three (USART1..USART3); the CH32V303RC and VC all eight, USART1..USART3
  and UART4..UART8. `device::has_usart(n)` is a mask, not a count.
- **The upper ports are not at the addresses their numbers suggest**:
  UART5 follows UART4 at 0x4000 5000, while UART6, UART7 and UART8 sit
  BELOW USART2, at 0x4000 1800, 0x4000 1C00 and 0x4000 2000 (tables 18-6
  to 18-9); their gates are PB1PCENR's bits 20 and 6..8 (3.4.8).
- **THE FOURTH PORT IS A USART ON THE CH32V203C8 ALONE.** Chapter 18's
  opening counts three USARTs and five UARTs for the whole family and
  then names the exception - "for CH32V203C8 the serial port 4 is a
  universal synchronous asynchronous receiver transmitter" - and that
  part's pin table agrees, bringing out USART4_CK, USART4_CTS and
  USART4_RTS. On the CH32V203RB and on every CH32V303 it is a UART4 with
  TX and RX alone, and UART5..UART8 are UARTs everywhere. So the
  synchronous clock, the smartcard and the flow-control pair are a PART
  fact (`device::usart_full`), and the verbs that configure them are
  REFUSED AT COMPILE TIME on an instance that is not full; the questions
  about them (`smartcard_enabled()`, `synchronous_enabled()`) answer
  false there. Measured on the CH32V203C8T6: the fourth port's CK pad
  carries a clock, and its vector is the one at that class's own tail.
  Measured on the CH32V303VCT6: UART5..UART8 each opened by its gate,
  its divisor read back, a byte read off its transmit pad by software
  and a frame banged into its receive pad through the pad's own pull,
  byte for byte, with no wire on any of them.
- **The pads are AFIO's columns** (tables 10-23 to 10-31), which the
  driver takes from the same table that programs the register - TX, RX,
  CK, CTS and RTS move together. UART4 reads a DIFFERENT TABLE per
  class: the column that starts at PC10/PC11 is the CH32V20x_D8's and
  the CH32V303's (table 10-26), and the CH32V203C8 is named in the
  other, whose default pads are PB0 and PB1 and whose second column is
  PA5/PB5/PA6/PA7/PA15. UART5..UART8 default to PC12/PD2, PC0/PC1,
  PC2/PC3 and PC4/PC5, with a second column each on ports A and B and a
  third on port E, which only the LQFP100 bonds. USART2's second column
  (PD5/PD6) belongs to the other classes and is refused on the D6; a
  column with no pad bonded on the package is refused too.
- **Two columns land on the debug port's own pads**: USART3's code 2
  (PA13/PA14) and UART8's code 1 (PA14/PA15). A transport naming either
  is refused by `init()` - false, nothing written - while the two-wire
  port is still the probe's; `Afio::disable_debug_port_until_reset()` is
  how a program gives the port up first, and it loses its probe until
  the next reset when it does. Measured on the CH32V303VCT6 with the
  probe attached: SW_CFG reads 0, the port alive, and both transports
  are refused with the field left as it was.
- **The vectors** are 53..55 for USART1..USART3 on every class; UART4 is
  61 on the CH32V20x_D6 and 66 on the D8, and on the CH32V303 UART4 and
  UART5 are 68 and 69 and UART6..UART8 87, 88 and 89 - the tail of that
  class's own table (device.hpp's Irq table and the crt's). Measured: a
  transmission complete reaches each of 68, 69 and 87..89 on the
  CH32V303VCT6.

### The frame and the divisor

Measured on the CH32V203C8T6 and the CH32V303VCT6 with the same
numbers, the instrument USART2 on its default pads: the block is one
register file on both series, and the bench suite's letters a..m run on
both parts.

- **The bit IS the divisor.** BRR counts peripheral clocks per
  sixteenth of a bit, so the whole register is pclk/baud and a start bit
  lasts exactly BRR peripheral clocks: measured to the ruler's own count
  at 9600, 115200, 921600 and 3 Mbaud. Every standard rate from 1200 to
  3 Mbaud comes out within ONE per mille of what was asked on both buses.
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

### The modes

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
  bits are what a bare board can judge; the wire is what it cannot -
  on either board, neither of which carries a resistor on that pad.
- **The flow-control pair** is the receiver's and the transmitter's own
  hands: CTS sampled high holds the next frame in the data register - TC
  never comes - and a low level lets it out at once; RTS is low while the
  receiver can take a frame, high while a received byte waits unread and
  low again when it is read. A change on the CTS line raises its flag in
  both directions and CTSIE carries it to the vector.

### The DMA requests are slots

On this family the channel IS the request ([dma.md](dma.md)), and on the
CH32V303 a request lands on one of TWO controllers, so each direction of
a port answers a SLOT - the controller and the channel. USART1 transmits
on DMA1's channel 4 and receives on 5, USART2 on 7 and 6, USART3 on 2
and 3, on every part that has them. UART4 is DMA1's 1 and 8 on the
CH32V203 (tables 11-5 and 11-6) and DMA2's 5 and 3 on the CH32V303
(table 11-3), whose UART5..UART8 are DMA2's too: 4 and 2, 6 and 7, 8 and
9, 10 and 11 (tables 11-3 and 11-4). An engine on any other slot is
refused at compile time. Measured on the CH32V303VCT6's crossed pair:
USART2's two engines on DMA1 against UART4's on DMA2, a message each way
and then four kilobytes each way at 921600 baud, every byte in order and
no error, overrun or fault counted at either end.

### The lot's four features (the CH32V303's)

Four things the chapter gives the CH32V30x_D8 - and there only "with the
penultimate sixth digit of the lot number not being zero", which no
register states and the part number does not carry - and none of them
the CH32V203's, where every verb that names one is refused at compile
time:

- **CTLR4 (18.10.8)**, a register of its own at offset 0x1C: CHECK_SEL
  holds the frame's parity bit at ONE (MARK) or at ZERO (SPACE) instead
  of computing it, and MS_ERRIE is the interrupt of a received parity
  bit that did not hold that level.
- **CTLR1's M_EXT (18.10.4)**: a word of seven, six or five data bits,
  where 00 leaves the word to M.
- **STATR's MS_ERR and RX_BUSY (18.10.1)**: the MARK/SPACE error, read
  and cleared as PE is, and a receiver in the middle of a frame.

A program cannot read its lot, and this stratum's rule for a lot's
register ([README.md](README.md)) is that the verb ASKS THE DIE. M_EXT is
a field of a register every die has, so `short_word()` writes it and
reads it back, and puts it back to zero where the die did not keep it.
CTLR4 is a register of its own, and an absent register can answer as a
MIRROR of another ([opa.md](opa.md)'s EXTEN_CTR2), so `ctlr4_present()`
reads the word FIRST - the real register holds nothing outside its three
bits, and a word with any other bit set is another register's, answered
no with nothing written - and then writes it only on a disabled port,
the three bits inverted, every other register of the block compared
before and after; when anything else moved, the block is put back
through its reset line and the program's words, and the answer is no.
The answer is kept per instance, and every CTLR4 verb asks it before it
writes a bit. MS_ERR and RX_BUSY are bits of a register that is there,
and read zero on a die without them.

**The CH32V303VCT6 the reference suite ran on has none of the four.**
CTLR4's address read 0 on USART2 and on UART4, the probe's write came
back 0 and moved nothing else in either block; M_EXT kept no code;
RX_BUSY read clear in every one of some 25000 polls while a frame
arrived at 1200 baud. And THE WIRE AGREES, which is what tells an absent
register from a write-only one: with CTLR4 stored raw, past the verbs, a
zero byte's low run stayed ten bits under even parity with MARK and nine
under odd parity with SPACE - the parity bit computed, as without it -
and with M_EXT's seven- and five-bit codes stored raw a zero word stayed
nine bits low, eight data bits and the start. It is the lot the other
lot-keyed registers of this die already said it is
([README.md](README.md)).

## Types and verbs

[brio/ch32vx03/usart.hpp](../../brio/ch32vx03/usart.hpp), two strata:

- The vocabulary: `UartBits` (seven, eight, nine DATA bits),
  `UartParity`, `UartStop` (the four codes), `UartFormat` and
  `uart_format_valid()`, `usart_ctlr1_format()` / `usart_ctlr2_stop()`
  (the register fields of a format), `uart_data_mask()`, `MuteWake` +
  `MuteConfig` (+ `mute_valid()`), `LinConfig`, `IrdaConfig` (+
  `irda_valid()`), `UsartSyncConfig`, `SmartcardConfig` (+
  `smartcard_valid()`), `usart_divisor(pclk, baud)` with
  `usart_divisor_valid()`, `usart_actual_baud()` and `usart_min_hz()`;
  the lot's vocabulary, `UsartMarkSpace` (off, mark, space) with
  `usart_ctlr4_check()` / `usart_mark_space_of()` /
  `usart_mark_space_valid()`, `UsartShortWord` (off, seven, six, five)
  with `usart_ctlr1_short_word()` / `usart_short_word_mask()` /
  `usart_short_word_valid()`, and `usart_class_has_lot_registers`; the
  instance facts `usart_base_for()`, `usart_bus_for()`,
  `usart_gate_for()`, `usart_irq_for()`, `usart_remap_of()`,
  `usart_remap_valid()`, `usart_column_for()` / `usart_pads_for()` (the
  five pads of a column, and the two a transport claims),
  `usart_column_on_debug_port()`, `usart_dma_tx_slot()` /
  `usart_dma_rx_slot()` (the controller and the channel a direction's
  request lands in) with `usart_dma_tx_channel()` /
  `usart_dma_rx_channel()` the channel alone.
- `Usart<n>` for n = 1..8 is the resource: `bus_clock()`, `reset()`,
  `remap(code)`, `enable()`/`enabled()`, `transmitter()`, `receiver()`,
  `configure(UartFormat, brr)` (the frame and the divisor written whole
  with the port disabled, refusing a format the word length cannot carry
  or a divisor out of range), `stop_bits()` (refused under IrDA),
  `set_brr()`/`brr()`/`actual_baud()`, `mute_mode()`/`mute()`/
  `unmute()`/`muted()`, `lin()`/`lin_off()`/`lin_enabled()`,
  `send_break()`/`break_pending()`, `half_duplex()` both ways,
  `irda()`/`irda_off()`/`irda_enabled()`, `prescaler()`/`guard_time()`,
  the full USART's `smartcard()`/`smartcard_off()`,
  `synchronous()`/`synchronous_off()`, `flow_control(rts, cts)` and
  `cts_interrupt()` - each refused at compile time on a UART - with
  `smartcard_enabled()`/`synchronous_enabled()`/`rts_enabled()`/
  `cts_enabled()` answering on every instance,
  `dma_transmit()`/`dma_receive()`/`data_address()`, the interrupt
  enables (`interrupts(mask)`, `rxne_interrupt()`, `txe_interrupt()`,
  `tc_interrupt()`, `idle_interrupt()`, `parity_interrupt()`,
  `break_interrupt()`, `error_interrupt()`),
  `status()`/`flag()`/`clear_flags()` (the write-zero flags alone)/
  `clear_by_read()`/`take_errors()`, `tx_empty()`/`tx_complete()`/
  `rx_ready()`, `write_data()`/`write_word()`/`read_data()`/
  `read_word()`; the lot's verbs, the CH32V303's alone -
  `ctlr4_present()`/`ctlr4_known()`, `mark_space()` both ways,
  `mark_space_interrupt()`, `mark_space_error()`, `receiving()` and
  `short_word()` both ways; and the constants `number`, `bus`, `irq`,
  `pads`, `dma_tx_slot`, `dma_rx_slot`, `dma_tx_channel`,
  `dma_rx_channel`, `is_full`, `has_lot_registers`. Every mode refuses
  the company the chapter forbids it.
- `Uart<n, P, rx_size, tx_size, format, TxEngine, RxEngine, remap,
  opts>` for n = 1..8 is the transport task, on that resource:
  `init(clock, baud)`, `isr()` (the edge), `write_byte()`/`read_byte()`/
  `write()`, `rx_pending()`/`tx_idle()`, the counters (`rx_overruns()`,
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
column of the CH32V203C8's full fourth port:

```cpp
constexpr brio::UartOptions link_opts{.rts = true, .cts = true};
using Link = brio::Uart<4, P, 64, 64, brio::UartFormat{.parity = brio::UartParity::even},
                        brio::NoDmaEngine, brio::NoDmaEngine, 1, link_opts>;
Link::init(clock, 9600);                          // TX PA5, RX PB5, CTS PA7, RTS PA15
```

An upper port of the CH32V303RC or VC, and its vector at the class's
tail:

```cpp
using Aux = brio::Uart<6, P>;                     // UART6, PC0/PC1
Aux::init(clock, 57600);
extern "C" BRIO_CH32_INTERRUPT void uart6_handler() { (void)Aux::isr(); }
```

The transport with both DMA engines, on the slots this instance's
requests are wired to - DMA1's for USART2, DMA2's for UART4 on the
CH32V303:

```cpp
using Loop = brio::Uart<2, P, 256, 256, brio::UartFormat{},
                        brio::DmaTxEngine<1, 7>, brio::DmaRxEngine<1, 6>>;
extern "C" BRIO_CH32_INTERRUPT void dma1_channel7_handler() { (void)Loop::dma_isr(); }
extern "C" BRIO_CH32_INTERRUPT void dma1_channel6_handler() { (void)Loop::dma_isr(); }

using Far = brio::Uart<4, P, 256, 256, brio::UartFormat{},
                       brio::DmaTxEngine<2, 5>, brio::DmaRxEngine<2, 3>>;
extern "C" BRIO_CH32_INTERRUPT void dma2_channel5_handler() { (void)Far::dma_isr(); }
extern "C" BRIO_CH32_INTERRUPT void dma2_channel3_handler() { (void)Far::dma_isr(); }
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

using Clocked = brio::Usart<4>;      // a full USART on the CH32V203C8 - a compile error elsewhere
Clocked::transmitter(false);
Clocked::receiver(false);            // CPOL, CPHA and LBCL want both clear
Clocked::synchronous({.clock_idle_high = false, .last_bit_clock = true});
// the CK pad is the caller's: brio::usart_column_for(4, 1).ck is PA6 there
```

The smartcard, whose verb takes the card's clock with it:

```cpp
U4::smartcard({.nack = true, .guard_time = 16, .clock_prescaler = 6});
// SCEN, NACK, GT, PSC, 1.5 stop bits and CLKEN; PCLK1 / 12 on the CK pad
U4::smartcard_off();                 // SCEN and its NACK; the clock is synchronous_off()'s
```

The MARK parity on a CH32V303, asked of the die with the port disabled:

```cpp
using U = brio::Usart<2>;
(void)U::configure({.bits = brio::UartBits::eight, .parity = brio::UartParity::even}, brr);
if (U::mark_space(brio::UsartMarkSpace::mark)) {    // false: this die's lot has no CTLR4
    (void)U::mark_space_interrupt(true);
}
U::enable(true);
```

## Bench findings

The reference suite is `test_vx03_serial`, at 144 MHz - PCLK2 144, PCLK1
72 - with USART2 as the instrument on its default pads and the console
untouched on USART1: 44 verdicts in `z` on the CH32V203C8T6, 47 on the
CH32V303VCT6, whose letters n..p are that series' own. Almost every
letter needs no wire, and three facts of this silicon are why: the
receive pad is a bit-banged transmitter (an input follows its own pull,
which the output data register moves in a few cycles), a pad driven by a
peripheral still reaches a TIMER CAPTURE input on the same pad (USART2's
TX is TIM2's channel 3, the fourth port's remapped CK is TIM3's channel
1), and the fourth serial port is a USART whose clock pad can be counted.
Every fact above with a number is this suite's measurement. Two letters
want a strap and detect its absence: the loopback PA2 to PA3, and the
crossed pair of USART2's pads with UART4's default ones - PA2-PB1 with
PB0-PA3 on the CH32V203C8T6's board, PA2-PC11 with PC10-PA3 on the
CH32V303 evaluation board - and the second is measured on both below.

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
  by edge as they arrive. The two channels' edge detectors are two, so a
  width is read to ONE COUNT of the timer: a 3 Mbaud start bit, 48
  counts, read 49 once on the CH32V303VCT6 (at 9600 in the same run,
  15001 for 15000), and the suite's tolerance is never tighter than that
  count.
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
  and no error flag on either side - on both parts.
- **The crossed pair's engines** (the CH32V303VCT6): four kilobytes each
  way at 921600 baud - 923076 really, the divisor 78 of a 72 MHz bus - in
  44378 us each way, against 44373 for the bits alone, so the two
  controllers feed the two ports back to back.

## Not covered yet

Driver gaps, each with its reason:

- **A single-wire bus with a peer**, and with the pull-up the chapter
  asks for: the mode is armed and its exclusions hold, but the wire is
  released between frames and nothing on either board holds it - a
  resistor to the supply on the TX pad is what would make the frame's
  timing and the receiver's own echo measurable.
- **The smartcard's data path**: the register face, the guard time, the
  NACK bit and the card clock are measured, but ISO 7816-3 itself - the
  answer to reset, the NACK a parity error provokes, the guard time
  delaying TC - wants a card and a socket on the desk.
- **The synchronous mode against a peer**: this side is the master and
  its CK pad is counted, so what a slave puts on RX in step with that
  clock is a second board's answer.
- **The synchronous mode and the smartcard on a CH32V303**: its fourth
  port is a UART4, and of its full USARTs USART1 is the console, USART2's
  CK pad PA4 is no timer's channel-1 input the clock letters count on,
  and USART3's CK PB12 is the SPI select wire of the evaluation board -
  a rig with a counter on the clock pad is what would measure them there.
- **USART3 on a wire**: the instance is on both parts, and its default
  column is PB10 and PB11 - the same two pads I2C2 answers on
  ([i2c.md](i2c.md)), which carry another link on the CH32V203C8's board
  and I2C1's two wires on the evaluation board. Nothing on either board
  speaks to a third serial port; it arrives with the first program that
  needs one.

Implemented but not bench-verified, each with what would measure it:

- **The lot's four features on a die that has them**: the MARK and
  SPACE parity on the wire and the far receiver's MS_ERR, the short
  words across the pair and RX_BUSY under a frame are letter o's, and
  the CH32V303VCT6 answered no to each; what would measure them is a
  CH32V303 of a lot whose penultimate sixth digit is not zero.
- **`error_interrupt()` (EIE, the FE/ORE/NE vector under DMAR)**: an
  error provoked on a line a receive engine is draining is what would
  raise it.
- **The transport's `remap` parameter on a column other than the
  default**: the resource's `remap()` is measured through the fourth
  port's second column, and the transport's own path is compiled and
  not run.
- **The debug-port columns once the port is given up**: the refusal is
  measured with the probe attached; a program that gives the port up
  and then brings UART8 up on PA14/PA15 is what would measure the other
  side of it - with no probe attached.
- **`rx_pending()`**: the surface the other strata share is compiled
  here and exercised by the family fixture, and no letter asks it.
