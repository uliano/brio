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
- **A read of STATR ARMS the receive clear; the next read of DATAR
  performs it on every flag standing at that read** - measured with
  frames banged into USART2's receive pad and channel 7 reading DATAR,
  the CPU never: a break's FE raised after a status read is gone once
  the channel has read the break's own frame, one raised with no status
  read since the last DATAR read stands (FE and IDLE both read 1), and
  clearing IDLE that way forgets the idle the clearing frame armed (one
  frame after an idle that was read raises no IDLE; two frames do). The
  CH32V203's USART answers the same; the STM32F4's, under the same words
  in its manual, clears only what the status read saw. With RXNEIE armed
  beside DMAR the vector is entered once a frame, RXNE already cleared by
  the channel and CNTR moved (measured, eight frames, eight entries).
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
  at 9600); TC's clear is a PAIR, a STATR read then a DATAR write, or a
  zero written (14.8.1), so a DATAR write that no status read preceded
  leaves the TC of the frame before standing, and a wait for TC right
  after it returns at once with the new frame still on the wire
  (measured: a run timed behind such a wait queued behind the rest of
  that frame, 37 thousandths over its wire time at 2400); IDLE rises once after a frame followed by a frame's worth
  of high line; PE once for a bad parity byte, which is still
  delivered; TXE with TXEIE re-enters the vector until it is disarmed,
  which is what the transport's ISR does when its ring runs dry.
- **DMA**: USART1 transmits on channel 4 and receives on 5, USART2 on
  6 and 7 (table 8-2, the channel IS the request - [dma.md](dma.md));
  an engine on any other channel is refused at compile time.
- **The reset values** are table 14-3's: STATR 0xC0, the rest zero.
- **The errors are read then cleared**: PE, FE, NE, ORE and IDLE by
  the STATR-then-DATAR read; RXNE, TC, LBD and CTS also by writing
  zero. The transport drops a byte that arrived with an error and
  counts it: from the host side, 64 frames with a parity bit sent into
  an 8N1 console came out as 30 bytes taken and 34 framing errors,
  none lost silently.

## The receive side's offer, and what the transport takes of it

| the chapter's offer | taken? | why |
|---|---|---|
| DATAR, one level: no FIFO | the interrupt receiver takes every byte at RXNE, one entry a byte | nothing deeper to batch |
| DMAR and a CIRCULAR channel (8.2.1's CIRC, [dma.md](dma.md)) | the receive engine | the bulk path: the channel writes the whole receive storage lap after lap and is never re-armed, util/ring.hpp's `HardwareRing` reading it |
| IDLE and IDLEIE (14.8.1) | the engine's burst edge | one frame after the last stop bit; the channel's read finishes its clear, so the vector waits for the next frame with IDLEIE disarmed |
| RXNEIE beside DMAR | the engine's wait for a frame | the channel still takes the byte and the vector learns one came - the edge of a burst of one frame; ORE raises it too, which is how an overrun with nothing left to take is found and cleared |
| EIE (FE, NE, ORE under DMAR) and PEIE | counted by the engine's vector | an error is counted when it rises, disarmed until the channel's next read clears it |
| the half and full marks of the channel's lap | the engine's edge with no silence | a stream that never pauses is told twice a lap |
| a receiver time-out | none on this block | |
| the LIN break flag | not taken | the transport carries no LIN; a break is the 0x00 frame with FE it is |
| TC written 0 (14.8.1) | at every transmit block's start | the channel's writes of DATAR run no part of TC's software clear |
| HDSEL | not a loop here | the receiver does not hear its own frames (above): no loop-back on this family, so the suites bang the receive pad |

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
  (the edge), `write_byte()`/`write_bulk()` (a run: as many as fit,
  its first byte pushed and TXEIE armed before the rest is copied and
  armed again behind it, or with an engine the run queued whole and
  the engine nudged once), `read_byte()`/`read_span()`/
  `consume()` (the receive run in place; with a receive engine
  `consume()` answers whether the run was intact), `rx_pending()`,
  `tx_idle()` (THE WIRE IS IDLE: the ring empty, no block in flight and
  TC set, TC cleared as every transmit block starts), `rx_skips()` (the
  gaps in the stream the receive ring hands out, never cleared -
  util/serial_port.hpp's epoch: without an engine the SkipRing's skips,
  one at the consumer's look after `isr()` dropped a flagged byte or a
  full ring refused one; with one the view's skips plus an overrun and a
  restart after a transfer error), the counters (`rx_overruns()` - a byte a full
  ring refused, or with an engine a lap or a held run the ring skipped -,
  `frame_errors()`, `parity_errors()`, `noise_errors()`, `hw_overruns()`,
  `clear_errors()`), `rebase()`, `set_baud(hz, baud)` (the link moved
  with the clock standing still, the ring drained first, false and
  nothing written when unreachable), `can_baud(pclk, baud)`,
  `min_hz_for(baud)`, `release()` (the vector off, the channels stopped,
  CTLR1 cleared, the block's reset pulsed - the vendor's DeInit, the one
  act that withdraws a DMA request the block holds on the CH32V203
  ([dma.md](dma.md)) - the gate closed, the pads released), the engine
  verbs `dma_isr()` (the ISR body of both channels, the controller's one
  flag register read once; its true is the receive edge, as `isr()`'s),
  `harvest()` (the same edge asked from the consumer's side, and a
  stopped ring started again; it reads neither STATR nor DATAR),
  `dma_faults()`, and `actual_baud()`/`divisor_for()`. `init()` puts the
  block through its reset line first. `write_bulk()` copies a run of 16
  bytes or more whose source and ring slot share their alignment with
  the runtime's memcpy, any other with its byte loop.
- THE RECEIVE ENGINE'S EDGE IS THE USART'S VECTOR'S, and the CPU never
  reads DATAR while the channel owns it: `isr()` runs two states over
  the clear measured above. WAITING FOR THE END (IDLEIE, PEIE, EIE): an
  idle line or an error is counted, its clear armed by that status read,
  the edge reported; then WAITING FOR A FRAME (RXNEIE alone), whose
  entry reads NO STATR (it would arm the clear the next frame's own read
  performs) but CNTR: a moved count turns the vector back and reports
  the edge again - the only edge a burst of one frame gets. A frame the
  channel NEVER takes (a channel a transfer error stopped, or one a held
  request froze) would leave RXNE re-entering the vector for ever: the
  entries that find the count unmoved - with no transmit engine, those
  with TXEIE down - are counted, and the 255th gives the channel up
  (`dma_faults()` counted, RXNEIE down, the edge reported) for the
  consumer's next look to stop and bind again from the storage's first
  element, as after a transfer error. AN OVERRUN WITH NOTHING TO TAKE
  re-enters that wait the same way: a channel starved past a frame time
  leaves ORE up and, once let go, takes the frame DATAR held, so with no
  frame after it RXNE is down and RXNEIE - which ORE raises on its own
  (14.8.4) - re-enters with nothing for the channel to move, until the
  bound gives a live channel up and the restart's EIE re-enters on the
  same ORE. So the wait's second entry with the count unmoved reads
  STATR once and, finding ORE without RXNE, reads DATAR - the clear of
  14.8.1 - and turns back to the wait for the end; with the count
  unmoved no DATAR read has come since the status read that began the
  wait, so the clear it armed is armed still and the two reads count
  nothing twice and lose no count, the overrun counted once (measured,
  below). A frame landing between the two loads loses its error flags
  to the clear, as the frame after any counted error does; on the
  CH32V203 and the CH32V303 the request it raised outlived the CPU's
  read and the channel still stored its byte
  ([../ch32vx03/usart.md](../ch32vx03/usart.md)), which this die has not
  been asked. The channel's half and full marks report it from
  `dma_isr()`. Two
  interrupts a burst, none a byte. WHAT THE CHANNEL'S READ BOUNDS is the
  counts, never the bytes: the frame after one whose error was counted
  loses its own (a run of errored frames counts every other one), so
  does a burst's first frame after an idle the vector saw, and a status
  read anywhere else - `tx_idle()`, the interrupt transmitter's entry -
  arms the clear for the next frame's error. `UartOptions`, the trailing
  parameter: `format` (seven data bits with parity or eight, with or
  without - nine is refused, the rings carry bytes), `half_duplex`
  (the TX pad as AF open drain, the RX pad untouched), `rts` and `cts`
  (the column's pads), and `rx_priority` - the receive engine's level,
  very_high by default ([../design/dma.md](../design/dma.md): DATAR holds
  one frame, and a copy that outranks the ring overruns it), refused on a
  port with no receive engine; the transmit engine arms at high.

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

A stream received without the CPU, its edge from two vectors:

```cpp
using Link = brio::Uart<2, P, 256, 256, brio::DmaTxEngine<6>, brio::DmaRxEngine<7>, 3>;
extern "C" BRIO_CH32_INTERRUPT void usart2_handler() {
    if (Link::isr()) { brio::post<LinkLines>(brio::RxActivity{}); }        // idle line, first frame
}
extern "C" BRIO_CH32_INTERRUPT void dma1_channel7_handler() {
    if (Link::dma_isr()) { brio::post<LinkLines>(brio::RxActivity{}); }    // the lap's marks
}
extern "C" BRIO_CH32_INTERRUPT void dma1_channel6_handler() { (void)Link::dma_isr(); }
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

The reference suite is `test_ch32_serial` (55 verdicts in `z`, five
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

- **THE RATE ASKED, which a start bit cannot tell** (the capture of
  a start bit is BRR to the cycle - the divisor measuring itself, which
  a divisor computed wrong would pass): letters b, c, i and t time a
  run of frames from the first store to TC on the ticker's cycle count
  against its wire time at the rate ASKED, HCLK over the baud named,
  and judge it -1 % to +3 %. The bare resource, polled one frame ahead
  of the shifter, at letter c's eight rates: 1003 thousandths at 2400
  (24 frames), 1000 at 9600, 76800, 115200 and 1.5 Mbaud, 999 at 460800
  and 921600 - the divisor's own 998 there, BRR 104 and 52 for 104.2
  and 52.1, the rounding's largest - and 1003 at 3 Mbaud (1024 frames).
  At 2400 the run is 240 bits, and the first frame's wait for the
  generator's next bit - half a bit at 9600 on letter j's lone frame -
  is most of the three. Every format of letter b, 96 frames at 9600,
  in 1000: 9.5 to 12 bits a frame, the half and the one-and-a-half
  stop and the parity bit inside the word as 14.8.4 spells it. The
  single-wire transport at 9600 in 1000. The transmit engine on
  channel 6: 256 frames at 9600 in 1000, 1024 frames - four laps of
  its ring, the next block started from the channel's vector - in
  1001 at 115200 and 1004 to 1005 at 3 Mbaud. The interrupt transmitter in
  1001 at 115200 and, moved live by `set_baud()`, in 1000 at 9600.
  The console's own USART1 through its transmit engine, on the
  CH32V006K8U6 and the CH32V003F4P6 alike: 272 frames at 115200 in
  1001 (`test_ch32_dma` letter e; BRR 417 for 416.7 is 0.8 thousandths
  slow), where the host reading the console would take a divisor a few
  per cent off without a sign. On the CH32V003F4P6 those two engines
  carry the whole of that suite (34 verdicts), every command typed
  arriving through the receive engine's ring.
- **Errors under the receive engine** (letter q, banged frames at
  9600): 64 data bytes with 7 banged breaks between them in a continuous
  stream - all 64 delivered intact and in order, the 7 breaks stored as
  the 0x00 frames they are, FE counted 7: no byte taken by a clear. Two
  breaks back to back count 1, three count 2. The interrupt receiver
  drops each break's frame and counts it (24 of 24, FE 6 for 6). A
  banged break wants its own stop bit before the next start bit: without
  one the receiver missed the next start and framed the rest wrong.
- **`tx_idle()` is the wire's** (letter r, TIM2 capturing the jumper):
  its first true lands 450 to 551 counts after the last stop bit starts
  at 115200 (a bit is 416) and 202 to 310 at 250000 (a bit is 192), on
  the interrupt transmitter and the transmit engine - TC at the stop
  bit's end, the rest the poll's turn on a 48 MHz core with a tick
  landing in it. At 1 Mbaud a bit is 48 cycles, under one turn of the
  poll, and the letter does not judge there.
- **The burst edge from the vector** (letter s, nothing polled): every
  burst of one frame told; a burst of 16 told 52770 cycles after its
  last stop bit (1.0 frame at 9600) with two USART interrupts and none a
  byte; four laps of the 256-byte ring with no silence read whole on the
  lap's marks.
- **An overrun at a burst's tail** (letter u, banged at 115200): a fill
  on channel 1 at the ring's own level, which wins the tie against
  channel 7 (8.2.1), starving the ring under a burst's last three frames,
  the vector held off until the channel had taken the frame DATAR held -
  ORE up, RXNE down, no frame after it. The vector entered THREE times,
  no channel was given up, the overrun and the gap were each counted
  once, the four frames before it and the one DATAR held arrived in
  order, and the next burst of eight arrived whole. THE CONTROL, the
  same image with the clear disabled: 256 entries - the overrun's and the
  stall bound's 255 - the live channel given up as a DMA fault, the
  frames before it thrown away by the restart, and the next burst lost
  to the second storm the restart's own entry began.
- **The rate verbs** (letter t): `can_baud()` yes at 3 Mbaud and no at
  3.2 from 48 MHz, yes at 733 baud and no at 732; `set_baud(9600)` on a
  live port gives a start bit of exactly 5000 cycles where 115200 gave
  417, a rate of 4 Mbaud is refused with the divisor left alone, and
  `release()` leaves CTLR1 at zero.
- **The cost** (`bench_ch32` letter u: USART2's transmit, the console's
  receive fed by `brio stress`): the interrupt receiver is one entry a
  byte, 247 cycles each between the stamps; the receive engine takes one
  interrupt a burst of 16 (the idle line, 295 cycles) where the BEFORE
  took none and the owner's poll a tick, and its edge comes 4630 cycles
  after the last byte landed at 115200 and 1315 at 460800 (1.1 and 1.2
  frames, the byte landing about half a stop bit before the line's end)
  where the poll's came anywhere in its millisecond. Transmit through
  the engine is wire-bound at 115200, 1 and 3 Mbaud (x 1.00 for 4096
  bytes); the interrupt transmitter is wire-bound at 115200, 1.08 at
  1 Mbaud and 1.40 at 3 Mbaud, its entry of about 133 cycles a byte
  longer than the 160-cycle frame leaves room for beside the tick.
- **Measured once while the round was written** (`bench_ch32`, before
  the transport's `init()` put the block through its reset line): a
  receive engine's session started after the plain console's on the
  same USART1 lost the first frame of the next burst, the channel
  counting 255 transfers of a 256-byte burst with no flag raised; with
  the reset at `init()` every later session took every frame.
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
- Exact error counts under the receive engine: bounded, not exact
  (above) - an exact count would need the CPU's read of DATAR, which the
  engine's rule forbids; declined.

Implemented but not bench-verified, each with what would measure it:

- `release()`'s reset pulse: written from the CH32V203's measurement and
  staged on no board of this family; the no-wire image of
  [dma.md](dma.md)'s first item - USART2 released without the pulse,
  then a TIM1-paced block on channel 6 - would measure it.
- The overrun clear's window on this die: whether a frame's request
  outlives a CPU read of DATAR, so that a frame landing between the
  clear's two loads keeps its byte; a fill starving the ring under one
  banged frame, the CPU reading STATR and DATAR before the fill ends and
  the channel's count read after it, would measure it.
- The receive ring over a DEAD channel: the stall bound is measured
  (letter u's control gave a live channel up at its 255th entry), the
  channel a held request froze is not; TIM2 gated by hand with its
  channel 2 request standing, then USART2's receive ring on channel 7
  fed a frame through its pad's pull, would stage it.
- The run verbs, `write_bulk()` and `read_span()`/`consume()`, on the
  CH32V003: compiled and counted; one run of the console on that part
  would measure them.
- The CTS flag and `cts_interrupt()`: the pair is driven and its hold
  measured on TC; the flag's own rise and vector are not counted.
- The transport's `rts`, `cts` and `half_duplex` options as a
  console's personality: the resource's verbs behind them are
  measured, the option'd `init()` path is compiled and the single-wire
  one heard a frame; a program run on such a port is what would prove
  the rest.
