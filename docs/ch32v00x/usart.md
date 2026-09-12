# USART (CH32V00x)

The USARTs of RM ch. 14 (two on the CH32V006, the CH32V003's one under
its ch. 12) - the asynchronous frame in every shape
the register description allows, the fractional baud generator, mute
mode with both wakes, LIN's break, single-wire half duplex, IrDA, the
flow-control pair, two DMA requests, every flag and interrupt - and
the interrupt-driven byte transport every brio console is, sitting on
that resource with its options costing nothing. Documents of record:
the CH32V00X reference manual V1.5 (14.2 for the frame and TE's idle
frame, 14.3 for the baud generator, 14.4 for half duplex, 14.5 for
IrDA, 14.6 for DMA, 14.7 for the interrupts, 14.8 for the registers;
3.4.4 and 3.4.7 for USART2's reset and gate; table 8-2 for the DMA
channels), the CH32V006 datasheet V2.0 (table 2-1-1 for the pads),
AFIO's tables 7-10 and 7-11 (the columns, [pin.md](pin.md)), and for
the CH32V003 its reference manual V1.9 (12.4 for the synchronous mode,
12.6 for the smartcard, 12.10 for its registers).

## What the silicon does

- **The STM32F1's USART minus two personalities on the CH32V006.**
  14.1 promises synchronous communication and smart cards; the
  register description has no such bits: CTLR2 bits 11:7 are reserved
  where the F1 keeps CLKEN, CPOL, CPHA and LBCL, CTLR3 bits 5:4 where
  it keeps SCEN and NACK, and GPR carries PSC alone, no guard time.
  Measured: written one, all of them read back zero.
  `Usart<n>::has_synchronous` and `has_smartcard` say so at compile
  time - and say the opposite on the CH32V003, whose USART is the
  F1's whole.
- **The CH32V003's synchronous mode** (its 12.4): with CLKEN the
  column's CK pad - PD4 on USART1's default column - carries one clock
  pulse per data bit while the transmitter shifts and at no other
  time; LBCL adds the last data bit's pulse (seven pulses per
  eight-bit frame without it, eight with it - measured), CPOL is the
  clock's idle level on the pad (measured), CPHA the receiver's
  capture edge; the three want TE and RE clear when written. THE
  RECEIVER SAMPLES ON THE OUTPUT CLOCK (12.4): with nothing answering
  on RX it collects one frame of the idle line per clocked byte
  (measured), so a program in this mode reads those frames as the
  full-duplex answer or keeps RE clear while it transmits alone. The
  mode excludes LIN, half duplex, IrDA and the smartcard.
- **USART2's registers answer in the PB1 address space (0x40004400),
  its gate and its reset are PB2's bit 13** (3.4.4, 3.4.7). Gated on
  PB1's bit 17, the F1's USART2EN, the block reads zero in every
  register (measured). USART2 exists on the CH32V005/006/007; on the
  CH32V006K8 its default column puts TX on the reset pin, so it lives
  on columns 1..6.
- **The frame is a word length plus a parity choice**: M gives 8 or 9
  bits INCLUDING the parity bit when PCE is set, so seven data bits
  exist only with parity and nine only without; STOP has four codes,
  1, 0.5, 2 and 1.5 bits, and the receiver takes all four (measured
  both ways: the transmitter's low run and stop gap on a timer
  capture, the receiver fed each format from a bit-banged line).
- **The bit is the divisor.** BRR counts peripheral clocks per
  sixteenth of a bit, so a start bit lasts exactly BRR cycles:
  measured to the cycle at eight rates from 2400 (20000 cycles) to
  3 Mbaud (16), the fractional divisor included (76800 = 39 and 1/16,
  625 cycles). Below 16 the generator has nothing to divide by and
  `configure()` refuses.
- **TE sends an idle frame first** (14.2): a byte written right after
  the enable waits a whole frame in the data register before TXE
  returns. Measured: 1145 us at 9600 for the first byte, 52 us once
  the idle frame is out.
- **HALF DUPLEX IS A BUS, NOT A LOOP.** With HDSEL the receiver listens
  on the TX pad through the alternate-function mux - a frame driven
  onto that wire from another pad arrives byte-exact with the pad in
  AF open drain and a pull-up on the line, and is not heard with the
  pad as a plain input - but the instance's own frames never reach
  its receiver: measured six ways (push-pull or open drain, the RX pad
  released, floating, pulled or handed to the function). The
  single-wire loop-back the F1 offers and the STM32G0 stratum uses as
  its suite's instrument is not on this silicon.
- **The receiver samples three times near the middle of the bit**, at
  about 13, 15 and 17 thirty-seconds (measured by walking a glitch
  along a banged bit): a glitch over one sample raises NE and the
  majority keeps the bit, over two it flips the bit AND raises NE,
  over all three it flips it in silence. A frame 3 per cent off the
  rate is taken clean, 6 per cent off raises FE or NE (14.3's floor of
  3 per cent holds).
- **The break** SBK sends is ten bits of low, thirteen in LIN mode
  (measured on the capture: 50000 and 65000 cycles at 9600); the
  detector raises LBD, and its interrupt once, on ten low bits under
  LBDL 0 and eleven under LBDL 1 - one bit short of each is a frame.
- **Mute mode**: with RWU set, frames back to back are not received;
  under WAKE 0 an idle line clears RWU and the next frame is; under
  WAKE 1 a nine-bit frame whose MSB is set and whose low four bits are
  ADD wakes the receiver, which then takes that frame and what
  follows, and sleeps through another node's. 14.8.4's two notes are
  real: a byte must have been received before an idle-line wake
  works, and RWU cannot be set under an address mark while RXNE
  stands (`mute()` refuses).
- **IrDA**: the encoder's pulse is 3/16 of a bit in normal mode (938
  cycles at 9600) and three periods of the prescaled clock in
  low-power mode (48 cycles with PSC 16); the decoder takes a
  bit-banged RZI frame - a zero is a pulse against the idle level -
  with the line idling either way, and with a pulse of 3/16 or of half
  a bit; with the line idling low the break flag rises beside the
  byte, and a change of the idle level needs a frame time before the
  decoder is clean again.
- **The flow-control pair**: CTS high holds the next frame back (TC
  does not come) and low lets it out; RTS is low while the receiver
  can take a frame, high while a received byte waits unread, low
  again once it is read.
- **The flags**: TXE returns within a bit of a write on an idle
  transmitter, TC at the end of the frame (1093 us for a 1042 us frame
  at 9600); IDLE rises once after a frame followed by a frame's worth
  of high line; PE once for a bad parity byte, which is still
  delivered; TXE with TXEIE re-enters the vector until it is disarmed,
  which is what the transport's ISR does when its ring runs dry.
- **DMA**: USART1 transmits on channel 4 and receives on 5, USART2 on
  6 and 7 (table 8-2, the channel IS the request - [dma.md](dma.md));
  an engine on any other channel is refused at compile time. Both
  engines of USART2 measured: sixteen banged frames harvested exact,
  a 256-byte ring moved in one run at the wire's pace (267 ms for 2560
  bits).
- **The reset values** are table 14-3's: STATR 0xC0, the rest zero.
- **The errors are read then cleared**: PE, FE, NE, ORE and IDLE by
  the STATR-then-DATAR read; RXNE, TC, LBD and CTS also by writing
  zero. The transport drops a byte that arrived with an error and
  counts it: from the host side, 64 frames with a parity bit sent into
  an 8N1 console came out as 30 bytes taken and 34 framing errors,
  none lost silently.

## Types and verbs

[brio/ch32v00x/usart.hpp](../../brio/ch32v00x/usart.hpp), two strata:

- The vocabulary: `UartBits` (seven, eight, nine DATA bits),
  `UartParity`, `UartStop` (the four codes), `UartFormat` and
  `uart_format_valid()`, `usart_ctlr1_format()`/`usart_ctlr2_stop()`
  (the register fields of a format), `uart_data_mask()`, `MuteWake` +
  `MuteConfig` (+ `mute_valid()`), `LinConfig`, `IrdaConfig` (+
  `irda_valid()`), `usart_divisor(pclk, baud)` and
  `usart_divisor_valid()`; the instance facts `usart_base_for()`,
  `usart_pads_for()`/`usart_column_for()` (TX, RX, CTS, RTS under a
  remap code), `usart_clock_for()`, `usart_irq_for()`,
  `usart_dma_tx_channel()`/`usart_dma_rx_channel()`,
  `usart_remap_valid()`.
- `Usart<n>` is the resource: `bus_clock()`, `reset()`, `remap()`,
  `enable()`/`enabled()`, `transmitter()`, `receiver()`,
  `configure(UartFormat, brr)` (refusing an invalid format or a
  divisor below 16), `stop_bits()` (refusing more than one under
  IrDA), `set_brr()`/`brr()`/`actual_baud()`, `mute_mode()`/`mute()`/
  `unmute()`/`muted()`, `lin()`/`lin_off()`/`lin_enabled()`,
  `send_break()`/`break_pending()`, `half_duplex()` both ways,
  `irda()`/`irda_off()`/`irda_enabled()`/`prescaler()`,
  `flow_control(rts, cts)` + `rts_enabled()`/`cts_enabled()`,
  `dma_transmit()`/`dma_receive()`, the interrupt enables
  (`interrupts(mask)`, `rxne_interrupt()`, `txe_interrupt()`,
  `break_interrupt()`, `cts_interrupt()`, `error_interrupt()`),
  `status()`/`flag()`/`clear_flags()` (the write-zero flags alone)/
  `clear_by_read()`, `tx_empty()`/`tx_complete()`/`rx_ready()`,
  `write_data()`/`write_word()`/`read_data()`/`read_word()`,
  `data_address()`; the constants `irq`, `dma_tx_channel`,
  `dma_rx_channel`, `has_synchronous`, `has_smartcard`; and
  `synchronous(UsartSyncConfig)` / `synchronous_off()` /
  `synchronous_enabled()` - CLKEN with CPOL, CPHA and LBCL, refused
  where the part has not the bits, while another mode is on, or while
  TE or RE is set; the CK pad is the caller's to hand over
  (`afio_usart1_pads(code).ck`). The modes refuse each other's
  company as 14.4 and 14.5 say.
- `Uart<n, P, rx_size, tx_size, TxEngine, RxEngine, remap, opts>` is
  the transport task, on the resource: `init(clock, baud)`, `isr()`
  (the edge), `write_byte()`/`read_byte()`, the counters
  (`rx_overruns()`, `frame_errors()`, `parity_errors()`,
  `noise_errors()`, `hw_overruns()`, `clear_errors()`), `rebase()`,
  the engine verbs `dma_isr()`, `harvest()`, `dma_faults()`, and
  `actual_baud()`/`divisor_for()`. `UartOptions`, the trailing
  parameter: `format` (seven data bits with parity or eight, with or
  without - nine is refused, the rings carry bytes), `half_duplex`
  (the TX pad as AF open drain, the RX pad untouched), `rts` and `cts`
  (the column's pads). Left at their defaults the options compile to
  the code they replaced: every image byte-identical (measured).

## How to use it

The console, as every app has it:

```cpp
using Serial = brio::Uart<1, P>;                 // USART1, PD5/PD6, 8N1
constexpr Serial serial;
Serial::init(clock, 115200);
extern "C" BRIO_CH32_INTERRUPT void usart1_handler() { (void)Serial::isr(); }
```

A second port with a frame and the flow-control pair, on a column:

```cpp
constexpr brio::UartOptions link_opts{
    .format = {.parity = brio::UartParity::even}, .rts = true, .cts = true};
using Link = brio::Uart<2, P, 64, 64, brio::NoDmaEngine, brio::NoDmaEngine, 3, link_opts>;
Link::init(clock, 9600);                          // TX PD2, RX PD3, CTS PA0, RTS PA1
```

A one-wire bus (a pull-up on the wire; what this node sends, it does
not hear back):

```cpp
constexpr brio::UartOptions bus_opts{.half_duplex = true};
using Bus = brio::Uart<2, P, 64, 64, brio::NoDmaEngine, brio::NoDmaEngine, 3, bus_opts>;
```

The chapter beyond the transport, on the resource - a LIN break and a
muted receiver waiting for its address:

```cpp
using U = brio::Usart<2>;
U::lin({.break_11bit = true, .break_interrupt = true});
U::send_break();                                  // thirteen bits of low, then the flag on the next one seen

U::mute_mode({.wake = brio::MuteWake::address_mark, .address = 5});
U::mute();                                        // asleep until a 9-bit frame carries 0x105
```

## Bench findings

The reference suite is `test_ch32_serial` (31 verdicts in `z`, four
of its letters on the jumper PD2 to PD4 that lends TIM2's channel 1 as
the ruler, one host-assisted letter outside `z` driven by `brio
stress`) on the CH32V006K8U6 at 48 MHz, with USART2 on column 3 as the
instrument and the console untouched on USART1. Every fact above with
a number is its measurement; the shape of the instrument is the
suite's header comment - the RX pad as a bit-banged transmitter paced
on the STK, the capture on the jumper as the ruler, no loop-back. On
the CH32V003F4P6 the suite is one group image of its synchronous-mode
letter alone (6 verdicts): with one USART, the console's, there is no
instrument for the rest, and every letter naming USART2 is compiled
out there.

- **The synchronous mode, with no wire** (CH32V003): the console's
  own USART1 with CLKEN, its CK pad PD4 read by TIM2's channel 1 off
  the same pad - sixteen bytes printed give 224 transitions (seven
  pulses a byte) without LBCL and 256 with it, CK idling low under
  CPOL clear and high under CPOL set, CPHA landing with the count
  unchanged, and no transition once the mode is off; the verb refuses
  with the transmitter live. On the CH32V006 the same letter proves
  the refusal and CTLR2 untouched.

## Not covered yet

Driver gaps, each with its reason:

- The smartcard (SCEN, NACK, the guard time): the CH32V003 has the
  bits and the CH32V006 has not; declined - no card and no reader on
  the desk, and the verbs are born with a card.
- The synchronous mode's receiver: it samples on the output clock
  (measured, above), but what it decodes wants a synchronous peer
  answering on RX; and the mode on a column other than the console's,
  whose CK pad would need a wire to a counter.
- A one-wire bus with a second node: the option is proven with a frame
  driven onto the wire from a pad and with the node's own frame not
  heard back; a peer on the wire is what a bus letter would need.
- USART2 on its other columns and USART1 on any but the console's:
  each column is a wire to a peer on its pads.

Implemented but not bench-verified, each with what would measure it:

- `error_interrupt()` (EIE, the FE/ORE/NE vector under DMAR): the
  receive engine runs with the errors counted at harvest; an error
  provoked from the banged line under an engine would measure the
  interrupt.
- The CTS flag and `cts_interrupt()`: the pair is driven and its hold
  measured on TC; the flag's own rise and vector are not counted.
- The transport's `rts`, `cts` and `half_duplex` options as a
  console's personality: the resource's verbs behind them are
  measured, the option'd `init()` path is compiled and the single-wire
  one heard a frame; a program run on such a port is what would prove
  the rest.
