# USART - the serial ports (STM32F4)

Documents of record: RM0090 Rev 22 ch. 30 (RM0390 Rev 6 ch. 25 and
RM0383 Rev 4 ch. 19 are its twins: one register description, three
manuals), the datasheets' alternate-function tables for the pads, and
the errata's USART items - ES0206 2.11.x, ES0298 2.13.x, ES0287 2.10.x:
"idle frame is not detected if the receiver clock speed is deviated",
"break frame is transmitted regardless of CTS", "guard time not
respected when data are sent on TXE events", "RTS is active while RE
or UE = 0", and their siblings. Two reach the transport: the first,
because the receive engine's burst edge IS the line's IDLE (below) and
the erratum has no workaround, and on the F446 ES0298 2.13.2 (PE
cleared by a write of DR in full duplex), because the task counts
parity errors while it may be transmitting - both staged from a foreign
sender ("Against a far board": the first reproduced in its own case,
one idle frame from a faster sender, and read entry for entry the same
on the far STM32G0, whose sheet lists no such item; the second
reproduced in the erratum's order and absent from the task). ES0298
2.13.3 (no PE for an address with a parity error that ends mute mode)
is staged too, and reproduces after a SOFTWARE mute alone. The rest
shape nothing in it (no break, no smartcard and no flow control in the
transport), each of which the mode's future user meets. Driver:
`stm32f4/usart.hpp` (`Usart<n>` the resource, `Uart<n, pins, rx_size,
tx_size, TxEngine, RxEngine, opts>` the task, the vocabulary
`UartFormat`, `UartOptions`, `UartPins`, `MuteConfig`, `LinConfig`,
`IrdaConfig`, `SmartcardConfig`, `UsartSyncConfig`, `UsartFlag`, the
divisor arithmetic), over the reserve's instance facts
(`stm32f4/device_tables.hpp`), with the engine slots filled from
[dma.md](dma.md) (`stm32f4/dma_engine.hpp` the empty slot's tag). The
family fixture is `test/family_stm32f4/usart.cpp` with the negatives
that refuse an absent instance, flow control on a UART, an engine off
the instance's cells of the request mapping, an engine on a part class
whose manual was not read, a receive ring longer than one lap of the
stream's count, a receive level on a port with no receive engine, and
one pad twice. Bench: the console apps
and the platform suite on the four boards, the engined console in
`test_stm32f4_dma`, the transport on USART6's single-wire loop in
`test_stm32f4_serial` (the Nucleo-F446RE) - and, under its second
all-key `t`, USART1 against the desk's STM32G0B1RE running `usart_peer`
("Against a far board" below) - its cost in `bench_stm32f4` letter u.
The instrument for a far board's suite is this stratum's `usart_peer`
("The peer" below).

## What the silicon does

**The classic STM32 USART**: SR/DR/BRR/CR1/CR2/CR3/GTPR - the block the
CH32V00x stratum drives under WCH's names, and NOT the STM32G0's. The
flags are cleared by READ SEQUENCES and not by a clear register (30.6.1):
PE, FE, NE, ORE and IDLE by reading SR then DR; TC by reading SR then
writing DR, or by writing 0; RXNE by reading DR; LBD and CTS by writing
0 to SR. The interrupt receiver's handler is written around those
sequences: it reads SR once, decides from that copy, and lets the DR read
do the clearing - which is also why an overrun without a byte still reads
DR.

**What a status read begins, the stream's read of DR finishes** - and
only for what that status read saw. Measured on the F446 with the
receive stream running and no read of DR by the CPU, the facts behind
`test_stm32f4_serial`'s letters d and f: a break's FE, seen by a read of SR, is gone
after the stream reads the next frame; a FE raised AFTER a status read
survives the stream's read of its own frame; a FE seen by a status read
and raised again by the next frame is cleared by that frame's read - the
same bit; and an IDLE cleared this way also forgets the idle the clearing
frame armed, so a burst of one frame after it raises no IDLE (two frames
do). The RM's own ORE note says the same of a frame arriving between the
two reads (25.4.3). Every rule of the receive engine's vector below comes
from those four measurements.

**The receive side's offer, and what the transport takes of it:**

| the chapter's offer | taken? | why |
|---|---|---|
| RDR, one level (25.4.3): no FIFO | the interrupt receiver takes every byte at RXNE, one entry a byte | the block has nothing deeper to batch; at 5.625 Mbaud a byte is 320 cycles and the interrupt receiver keeps up with the console's and the tick's handlers beside it, an overrun counted when a longer one holds it off |
| DMAR and a circular stream (25.4.13, dma.md) | the receive engine | the bulk path: no entry a byte, no re-arm between laps |
| IDLE and IDLEIE: an idle frame after a frame (25.6.1) | the engine's burst edge | the end of a burst one frame after its last stop bit; the stream's read finishes its clear, so the vector waits for the next frame with IDLEIE disarmed |
| RXNEIE beside DMAR | the engine's wait for a frame | RXNE is the stream's request and the interrupt both: the stream still takes the byte and the vector learns one came - the only way to know the clear finished, and the edge of a burst of one frame |
| EIE (FE, NE, ORE under DMAR) and PEIE | counted by the engine's vector | an error is counted when it rises, then disarmed until the stream's next read finishes its clear |
| the half and full marks of the stream's lap (dma.md) | the engine's edge with no silence | a stream that never pauses is told twice a lap |
| a receiver time-out | none on this block | the STM32G0's RTOF is a later generation's |
| LBD, the LIN break flag | not taken | the transport carries no LIN; a break reaches the receiver as the 0x00 frame with FE it is, and LBD rises beside it whether or not LIN is on (measured) |
| HDSEL, single-wire half duplex | the loop of every suite letter | the receiver hears its own transmitter; this block has no loop-back bit |
| the bit-band alias of CR1 (RM0390 2.2.5, PM0214 2.2.5) | the single-bit interrupt enables | one store, atomic against a handler that rewrites CR1's other bits - the engine's vector does |
| TC, written 0 to clear (25.4.13 step 6) | at every transmit block's start | the stream's writes of DR do not run TC's software sequence; tx_idle() must not answer from the frame before the block |

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
register value is fCK / baud in sixteenths with no field arithmetic; at
OVER8 = 1 fCK / baud is USARTDIV in EIGHTHS, the fraction three bits and
bit 3 clear (30.6.3), for twice the reachable rate at half the
receiver's tolerance - the manual's own example, 8 MHz and 9600 baud,
is BRR 0x681, pinned in the family fixture. A USARTDIV below 1 - 16
sixteenths, 8 eighths - is refused: fCK / 16 and fCK / 8 are the tops.

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
  `irda(IrdaConfig)` (normal mode at PSC 1 alone, `irda_valid` - GTPR's
  25.6.7)/`irda_off`/`irda_enabled`;
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
  half_duplex, rts, cts, rts_pin, cts_pin, tx_speed,
  single_wire_push_pull}` - the defaults are the console's. In half
  duplex the TX pad is open drain on its pull-up unless
  `single_wire_push_pull`, which drives it push-pull while a frame goes
  out (the transmitter releases the pad between frames, 25.4.10, and the
  pull-up holds it): for a line with no other driver, because the open
  drain rises on the pull-up alone (letter a) - and `rx_priority`, the
  receive engine's level in its controller's arbitration, `very_high` by
  default (below), refused at compile time on a port with no receive
  engine.
- `Uart<n, pins, rx_size = 64, tx_size = 256, TxEngine = NoDmaEngine,
  RxEngine = NoDmaEngine, opts = {}>` - `init(clock, baud, format = 8N1)`
  (the divisor from `apb_hz(clock, on_apb2)`, false when unreachable or
  for nine data bits), `isr()` (the vector's body: the RX edge as its
  return), `write_byte`, `write_bulk`, `read_byte`, `read_bulk`,
  `read_span` and `consume` (the receive run in place), `rx_pending`,
  `tx_idle` (THE WIRE IS IDLE: the ring empty - an engine's block holds
  its run there until its completion - and SR.TC set, cleared as every
  transmit block starts), `rx_skips` (the gaps in the stream the receive
  ring hands out, never cleared - util/serial_port.hpp's epoch: without
  an engine the SkipRing's skips, one at the consumer's look after
  `isr()` reported a byte refused by a full ring, dropped for FE or PE,
  or an overrun; with one the view's skips plus an ORE and a restart
  after a transfer error, an FE frame being stored),
  the counters `rx_overruns`,
  `frame_errors`, `parity_errors`, `noise_errors`, `hw_overruns` - each
  SATURATING at 255, so a storm of errors never reads as a few -
  `clear_errors`, `rebase(hz)` (the ClockUser verb: `hz` is SYSCLK and
  the bus rate is DERIVED from it with `apb_hz_at`, not read back from
  the RCC, because a dynamic clock fans the new rate out BEFORE the
  prescalers move), `set_baud(hz, baud)`, `divisor_for`,
  `min_hz_for`, `can_baud`, `actual_baud(fck)`, `kernel_hz<Clock>()`,
  `release()` (the streams stopped, the interrupts off, UE clear, the
  block's RCC RESET pulsed before its gate closes - the release contract:
  a raised request is held through a gated clock on this family,
  [dma.md](dma.md)); with engines, `dma_isr()` (the streams' vectors' body,
  whose true is the receive edge as `isr()`'s), `harvest()` (the same
  edge asked from the consumer's side, below) and `dma_faults()`. Refused at compile time: invalid or coincident pads,
  an engine off the instance's request cells, flow control on a UART, a
  flow pad missing, and with a receive engine a ring above 32768 bytes.
- THE RECEIVE ENGINE ARMS AT `rx_priority`, `very_high` by default, and
  the transmit engine at `high` (docs/design/dma.md's rule): DR holds one
  character, so a starved receive stream overruns, and a starved transmit
  one leaves the line idle a while. On this controller the level decides
  correctness: a ring armed low beside three copies at very_high on its
  own DMA2 lost up to half of what came, the default ring nothing beside
  four ([dma.md](dma.md), `test_stm32f4_serial` letter g).
- WITH A TRANSMIT ENGINE THE MASK COVERS THE CLAIM AND NOTHING ELSE. Two
  contexts start blocks - a print in the loop and the completion handler
  starting the ring's next run - so the decision is masked: the ring's
  run read and the engine's test-and-set, twelve instructions in the
  listing. The block start (four stores and one read of the stream,
  dma.md) runs with interrupts on, because a claimed stream is not
  running and cannot complete under it.
- WITH A RECEIVE ENGINE THE RECEIVE RING IS THE STREAM'S. The engine
  runs in its circular shape over the whole receive storage, bound once
  at `init()` and never re-armed (dma.md), and the consumer verbs -
  `read_span`/`consume`, `read_byte`, `read_bulk`, `rx_pending` - read it
  through util/ring.hpp's `HardwareRing`, whose producer index is the
  stream's SxNDTR and the lap count its vector keeps. So the ring is a
  power of two and one lap of SxNDTR, 32768 bytes at most. A byte is in
  the ring the moment the stream has stored it, whoever asked; what the
  consumer can lose is a LAP it did not keep up with, which the view
  counts in `rx_overruns()` - one count a lap, up to a ring's worth of
  bytes skipped - and a consumer looking once a millisecond keeps up when
  the ring holds a millisecond of the line (92 bytes at 921600). The ring
  must be read from one context, the consumer's.
- THE RECEIVE ENGINE'S BURST EDGE IS THE USART'S VECTOR'S - and the
  CPU never reads DR while the stream owns it. `isr()` runs a two-state
  machine over the clears (the header's text, the measurements above):
  waiting for the END of a burst with IDLEIE, PEIE and EIE armed, an
  idle line or an error is counted, its clear begun by that status read,
  the edge reported, and the vector turns to waiting for a FRAME -
  those three disarmed, RXNEIE armed; the next entry that finds SxNDTR
  moved knows the stream's read finished the clear, counts what stands
  as a later frame's error, and turns back, reporting the edge again -
  the edge of a burst of one frame, which raises no idle of its own. The
  stream's half and full marks report it from `dma_isr()`. Two USART
  interrupts a burst, none a byte; the edge one frame after the last
  stop bit. The gate is the interrupt receiver's: true once per
  idle-to-busy transition of the consumer, re-opened by a look that
  finds the ring empty (and looks again, so a byte landing between is
  in the run or raises the edge).
- AN OVERRUN WITH NOTHING TO TAKE IS CLEARED BY THE VECTOR. A starved
  stream that reads DR at last before any status read saw the ORE leaves
  ORE up with RXNE down, and RXNEIE, which ORE raises on its own (RM0390
  25.6.4), re-enters the wait for a frame back to back with nothing for
  the stream to move - measured: the core held by the vector until the
  next frame, at a message's tail for ever (SR 0xD8, RXNEIE armed, the
  board stopped). So an entry that finds the stream unmoved and its status
  read showing ORE without RXNE reads DR - the clear that read began, no
  byte the stream was owed - counts the overrun unless the entry that
  began the wait did, and turns back to waiting for the end: one compare
  more on the unmoved entry. A frame completing between the two loads is
  the one the DR read takes, inside an overrun already.
- WHAT THE CHANNEL'S READ BOUNDS. A frame whose error rises while its
  flag from the frame before stands (seen, not yet cleared) is cleared by
  its own read unseen: a run of errored frames counts every other one,
  errors separated by a clean frame count each (letter d). And A STATUS
  READ ANYWHERE ELSE is the first half of the same clear: one landing
  between an errored frame's flag and the stream's read of that frame -
  a few cycles - clears it before the vector counts it, which a thread
  polling SR in a tight loop does to three breaks in ten (letter d).
  `tx_idle()` reads SR only once the ring is empty. No byte is ever
  lost to either: they bound the COUNTS.
- `harvest()` IS THE SAME GATE FROM THE CONSUMER'S SIDE: true when the
  ring holds bytes and the consumer has found it empty since the last
  true, from a vector or from here - so an owner that still asks is told
  nothing twice. It reads neither SR nor DR, and it binds again a stream
  a transfer error stopped (as every read verb does: the restart is the
  consumer's, the view being its).
- `write_bulk()` COPIES WITH memcpy where it pays: a run of 16 bytes or
  more whose source and ring slot share their alignment modulo the word
  takes the runtime's word path, any other the four-instruction byte
  loop (memcpy would run the same loop behind a call).

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
brio::NoDmaEngine, brio::NoDmaEngine, fast>`. A stream received without
the CPU, its edge from three vectors - the USART's and the two streams'
(dma.md's example has the cells):

```cpp
using Link = brio::Uart<6, link_pins, 256, 256, brio::DmaTxEngine<2, 6, 5>,
                        brio::DmaRxEngine<2, 1, 5>>;
extern "C" void USART6_IRQHandler() {
    if (Link::isr()) { brio::post<LinkLines>(brio::RxActivity{}); }       // idle line, first frame
}
extern "C" void DMA2_Stream1_IRQHandler() {
    if (Link::dma_isr()) { brio::post<LinkLines>(brio::RxActivity{}); }   // the lap's marks
}
extern "C" void DMA2_Stream6_IRQHandler() { (void)Link::dma_isr(); }      // transmit blocks
``` A mode beyond the
transport - a LIN break, a muted receiver - through `brio::Usart<1>`'s
verbs on the same instance.

## Bench findings

- The console runs byte-exact on the four boards at 115200 8N1:
  USART2 on PCLK1 45 MHz with BRR 391 (115089 baud) on the
  Nucleo-F446RE, USART1 on PCLK2 90 MHz with BRR 781 (115236) on the
  DISC1, USART1 on PCLK2 100 MHz with BRR 868 (115207) on the black
  pill, USART3 on PCLK1 45 MHz with BRR 391 (115089) on the
  32F469IDISCOVERY - the `ERR` verb reporting every counter zero after
  the platform suite's 36 verdicts and the console session.
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
- The engined console (`test_stm32f4_dma` letters l, m, n and u, the
  F446's USART2 on DMA1 streams 6 and 5): 575 bytes leave through the
  transmit stream in 50 ms where the line's own time is 49, no block
  thrown away. A block start masks the claim alone - from the `cpsid` to
  the `msr PRIMASK` twelve instructions, the ring's indices, the engine's
  flag and its store - where the whole stream start ran masked before,
  some 650 cycles of interrupt latency on the first byte of every run.
- The receive side over the circular stream takes every byte the host
  sends, at every rate the ST-LINK's bridge carries: `brio stress`
  pumping its xorshift into a ring of 256 bytes for a second, the
  consumer looking once a millisecond, 10688 bytes of 10688 at 115200,
  30820 of 30820 at 460800 and 32200 of 32200 at 921600 - no gap, no lap
  missed, no ORE. The same receive as a run re-armed by `harvest()` lost
  386, 3453 and 3495 of them in 73, 203 and 217 gaps, one ORE each - the
  bytes that arrived while a filled run waited for the next look
  ([dma.md](dma.md)). On the half-duplex echo the circular ring returns
  128 of 128 where the run returned 127, with no flag for the missing one.
- `harvest()` costs 57 core cycles with nothing new and 84 with bytes
  waiting: the restart flag, one look of the view and the gate, no
  register of the USART. The vectors' edge is measured both ways in
  `test_stm32f4_dma` letter n: the first bytes reported, not again to a
  consumer that has not drained, `harvest()` asked after them answering
  false, and the next bytes reported once the consumer has drained; a
  consumer asleep for three laps is told once, finds one overrun counted
  at its first look, nothing offered, and the bytes after it whole. And
  `brio stress` (letter u) with the consumer looking at most once a
  millisecond and only after an edge reads every byte at 115200, 460800
  and 921600: 10688, 30820 and 32200 of as many.

**The single-wire loop** (`test_stm32f4_serial`, USART6 on PC6): 256
bytes back whole and in order through both receivers at 115200, 1 Mbaud
and 5.625 Mbaud (APB2 / 16), and through the engine at 11.25 Mbaud
(OVER8 with ONEBIT, APB2 / 8 - BRR 0x10), 1024 frames leaving in their
wire time; every frame format of the task - 8N1, 8E1, 8O1, 7E1, 7O1, 8N2
- byte-exact at 1 Mbaud, a seven-bit frame's eighth bit being its parity
bit as the stream stores it (DR's MSB, 25.6.2; the interrupt receiver
hands it on too); the open drain on the internal pull-up alone whole at
1 Mbaud and 31 of 64 at 2.8125, and whole at 2.8125 where the pad carries
the bench's 2.2 kOhm I2C pull-up besides.

**A loop cannot tell a wrong rate**: both ends share the divisor, so a
divisor twice too large is a loop twice too slow and every byte still
comes back. Letters a and c therefore time 1024 frames out through the
transmit engine against their wire time - 10 x 1024 bit times - and judge
it within three per cent; the OVER8 arithmetic is also pinned against
the manual's example at compile time.

**Errors under the engine** (letter d): 120 data bytes with 14 breaks
between them, each followed by a clean frame - all 120 delivered intact
and in order, the 14 breaks stored as the 0x00 frames they are, FE
counted 14: no byte taken by a clear. Two breaks back to back count 1,
three count 2. Ten breaks under a thread polling SR as fast as it can:
7 counted. The interrupt receiver drops each break's frame and counts
it (40 of 40 data bytes, FE 10 for 10).

**The ring against copies on its own controller** (letter g, DMA2): the
default ring loses nothing beside four copies at very_high at 5.625 and
11.25 Mbaud; armed low (`rx_priority`), it loses nothing beside two and
a third to a half of its frames beside three or four, the overrun count
saturating at 255; the vector is quiet after every starved leg (the
overrun with nothing to take, above). The table is [dma.md](dma.md)'s.

**`tx_idle()` is the wire's** (letter e): its first true lands 1594..1603
and 1598..1604 cycles after the last stop bit's rising edge on the pad at
115200 (a bit is 1562) - interrupt transmitter and transmit engine - and
202 and 218..221 at 1 Mbaud (a bit is 180), four runs alike, the edge's
EXTI latency taken off: within a quarter bit of TC rising at the stop
bit's end, on both paths. The latency is the MEDIAN of seven software
triggers of the same line, 34 cycles (34..39): one trigger alone met the
SysTick's entry or a flash line's fill now and then and read the
interrupt transmitter bimodal run to run (215 or 338..348 cycles), and
the console is drained first (its transmit interrupt, entered as the
stop bit rises, once read the edge 22 cycles early).

**The burst edge from the vector** (letter f, nothing polled): every
burst of one frame told; a burst of 16 at 1 Mbaud told 1954 cycles after
its last stop bit (1.0 frame) with two USART interrupts and none a byte;
four laps of a 64-byte ring with no silence read whole on the lap's
marks, nine stream interrupts.

**The cost** (`bench_stm32f4` letter u on the same loop; each line raw,
the instrument's 4 cycles a stamp pair in every isr): the receive engine
takes 2 USART interrupts a burst and 1 stream interrupt a half lap; a
16-byte burst at 115200 costs 1702 busy cycles (the BEFORE, the owner's
poll a tick, 1716 and the edge 0.51 ms after the last stop bit), the
edge comes 15904 cycles after it (1.0 frame), 2000 at 1 Mbaud (1.1),
508 at 5.625 Mbaud (1.6, the handler's 197 cycles six tenths of a frame
there). The interrupt receiver is one entry a byte, 90 cycles each; at
5.625 Mbaud it has one frame of margin, and in the bench app, whose
metered handlers sit beside it, it took a hardware overrun - one byte of
256 - in two runs of four, where the suite's unmetered loop reads it
whole. Each
of the engine's two entries is about 197 cycles between the bench's
stamps, the app's glue included: the idle-line one about 60 instructions
and seven register accesses (SR, SxNDTR twice, CR1 and CR3 read and
written), the first-frame one about 45 and eight. Transmit through the engine is wire-bound (x 1.00 at 115200
and 1 Mbaud, 1.01 at 5.625 Mbaud for 4096 bytes, a block's completion
and restart a frame's gap every 255 bytes), at 0.6 per cent of the core
at 1 Mbaud (busy 46048 of 7.38 M cycles for 4096 bytes) against 11.6 per
cent through the interrupt transmitter (858022).

**Against the vendor** (ST's HAL v1.8.5, `HAL_UARTEx_ReceiveToIdle_DMA`
in circular mode on the same loop and bursts, a scratch program): the
edge at the same frame after the last stop bit (15881, 1985 and 497
cycles at the three rates against brio's 15904, 2000 and 508), ONE USART
interrupt a burst against brio's two, 188 cycles for its idle-line
handler and callback against brio's two entries of about 197 each -
brio pays one interrupt more a burst. The difference is the read the HAL
makes and brio does not: `__HAL_UART_CLEAR_IDLEFLAG` reads SR then DR
while DMAR is set, and a frame completing between the two reads is taken
by the CPU and lost to the stream without a flag; the HAL also treats any
receive error under DMA as fatal and aborts the reception. A gap above a
fifth, named: the price of never reading DR.

### Against a far board: the STM32G0B1RE's `usart_peer` (the second all-key `t`)

The desk pairs the Nucleo-F446RE's USART1 with the STM32G0B1RE's,
CROSSED - PA9 (AF7) to its PA10, PA10 from its PA9 - the far board
running the stm32g0 `usart_peer` (firmware 0x0301), driven IN BAND with
the protocol every architecture on the link compiles
(`avrdx/src/apps/usart_link.hpp`). Letters `1` to `9`, `0`, `r` and `s` of
`test_stm32f4_serial` run under the second all-key `t` and never under
`z`, which runs with the far board on its dark console. The far peer's
core is an HSI16 it TRACKS against its LSE crystal (docs/stm32g0/
clock.md, "The core against the LSE, tracked"), so every rate it times
is crystal against crystal - and every rate it SENDS it states (a
`send` or `blast` Report's aux16: its divisor on its tracked core). Score:
**142 of 142**, what this USART has no word for SKIPPED with its reason.

- **The rendezvous and the refusal path** (`1`): ping, IDENT (kind 0x03,
  another die, a crystal-referenced time base), a bad checksum NAKed, a
  desync recovered, a synchronous client refused and the peer at once in
  command mode again.
- **Two crystals**: three gapless runs of 40000 frames at 1 Mbaud (90
  MHz / 90, exact) timed by the far chip, judged to agree within 10 ppm
  - a bound DERIVED below, "The rendezvous's bound" - and measured
  within 2: 75 figures over 25 runs between -9 and -6 ppm, the ST-LINK's
  8 MHz against the far LSE, within their tolerances; the far chip's own
  1 Mbaud blast on this DWT +2999 ppm against the +2989 it states (64 of
  its 64.2 MHz core) less the clocks' -8: 2 ppm apart.
- **The baud matrix** (`2`) on the two dividers' COMMON ladder: this
  USART divides 90 MHz by any whole k from 8 (OVER8's three-bit fraction
  counts eighths), the far one 64 MHz by a whole n (its OVER8 adds no
  resolution), and they meet where k / n = 7 / 5 - above 2 Mbaud at
  2.56, 3.2, 4.267 and 6.4 Mbaud, this end 0.45 % above the rate and
  0.15 % from the far end. At 9600, 115200, 921600, 1, 1.5, 2 Mbaud and
  those four: 16 frames echoed by the far chip, every one equal; this
  transmitter timed by the far receiver over a thousand frames or 25 ms
  (864 at 9600) within 200 ppm of its divisor's error and the clocks'
  (measured within 25 ppm at every rung: 921600 -3504 against -3508 +
  -8, 6.4 Mbaud +4456 against +4464 + -8); the far blast whole on the
  polled receiver, its rate within 200 ppm of the far divisor's stated
  one less the clocks' (measured within 30: 6.4 Mbaud +2753 against
  +2747 + 8, 4.27 Mbaud +2913 against +2894 + 8); and
  1024 frames whole through the interrupt receiver and through the
  receive engine (DMA2 stream 2 channel 4) at every rung - OVER8's task
  above 5.625 Mbaud, the interrupt receiver keeping up at 6.4 Mbaud, 281
  cycles a frame.
- **The frame matrix** (`3`) at 115200: 8N1, 8E1, 8O1, 7E1, 7O1, 9N1,
  8N2, 8N0.5 and 8N1.5 echoed by the far chip; 7N1, 6E1 and 9E1 SKIPPED
  (M counts the parity bit: no word length for them here); the half stop
  bits timed as LENGTHS by the far receiver, 19 and 21 half bits within
  25 ppm of an 8N1 run.
- **The receiver's errors from a foreign sender** (`4`): odd parity into
  an even receiver, PE on each and the data readable; THE FE ONSET with
  the far divisor 0..8 % off 9600, back to back: clean within 4 % either
  way, FE from +5 % and -6 % (25.4.5's 3.75 % at OVER16 with no
  fraction, NE rising from 4 %); a run of 8 read late - ORE, DR keeping
  the run's first frame, the rest lost (one data register); THE START
  BIT'S VOTE, one low pulse of w ticks on the idle line at 9600 (a
  sixteenth is 156): nothing under 1350, a start with NE from 1350 to
  1500 and a clean start from 1550 with ONEBIT clear, a start from 1350
  and NE at no width with ONEBIT set; a break of 20 bit times one frame
  with FE and data 0x00, and LBD SET WITH LIN MODE OFF as well as on -
  the detector runs whatever LINEN says (25.6.1's description of the bit
  ties it to no mode).
- **ES0298 2.13.2 STAGED AND REPRODUCED** (`5`): 64 odd-parity frames
  into this end at 8E1 while it transmits back to back. Polled in the
  ERRATUM'S ORDER - an SR read and a DR write every turn, then the SR
  read the receive is decided on - PE was seen on 11 and 16 of 64 in two
  runs; in the WORKAROUND'S (one SR read, the DR read, then the write)
  64 of 64. THROUGH THE TASK, transmitting the whole time, every one of
  the 64 counted and none delivered: the vector's one SR read decides the
  receive before the transmit writes DR, which is the workaround's order
  by construction.
- **The far engine's shapes** (`6`), the STM32G0B1RE's TIM1: where a
  start is believed - eight low pulses a width at 115200, nothing under
  half a bit, 5 of 8 taken at 110 ticks (8.4 sixteenths) with NE, every
  one from 117, clean from 130 at OVER16 and 156 at OVER8; the
  operational range of a gapped 0x00 frame - decoded from -50 to +50 per
  mille at 9600 and 115200, broken by FE slow and by 0x80 fast, NE first
  from +-45..50 per mille: 25.4.5's 3.41 % inside both ways; one
  stretched cell of 0x55 at 115200 - 30 % either way decoded, 45 % not
  for the start cell, the stop cell free.
- **ES0298 2.13.1 STAGED, NOT REPRODUCED** (`7`): twenty bursts of
  sixteen frames at 50000 baud from a far end 0 to 3 % FASTER, gaps of
  200 us (one frame), 300 us and 1 ms: the polled receiver saw IDLE once
  a burst in every leg, and THE RECEIVE ENGINE'S CONTRACT HOLDS - its
  edges the same 39 for 20 bursts at every gap and offset (each burst's
  first frame and its IDLE, the very first frame's merged with the
  start), no IDLE edge missed. The erratum's case is an idle frame and
  no more followed at once by a fast sender's start bit; a gap the far
  peer can time in 100 us steps reaches one frame and a few
  microseconds, and the edge held there. THE ERRATUM'S OWN CASE, drawn by
  the far engine with the gap stated in ITS bit times (the `stage` op):
  sixteen frames, an idle of 9.5 to 11 sender bits after each, the
  sender exact, 1 % and 3 % fast. From the exact sender IDLE follows
  every frame from ten bits and none at 9.75; from 1 % fast from 10.25
  of its bits, from 3 % fast from 11 - ONE IDLE FRAME AND THEN A
  CHARACTER FROM A SENDER 3 % FAST IS NOT TOLD, nor ten and a half of its
  bits (10.2 of this receiver's, past the exact sender's threshold):
  2.13.1 REPRODUCES. And the far STM32G0's letter `9` reads THE SAME
  TABLE, entry for entry, where ES0548 lists nothing: the receiver
  counts its idle frame from the stop bit as IT sampled it, which a
  faster sender's frame ends ahead of by the error over the frame -
  every USART's geometry, the F446's sheet the one that names it. The
  receive engine's contract (a burst told within two frames) is the
  one-frame gap above, which holds.
- **The ring under copies, with a real sender** (`8`): letter `g`'s
  arrangement on USART1's receive engine (DMA2 stream 2), 4096 frames
  back to back from the far chip at 4.27 and 6.4 Mbaud: at its default
  level the ring loses nothing beside up to four very-high copies, word
  or byte beats; ranked low, two copies starve nothing and THREE do -
  3299 of 4096 lost at 4.27 Mbaud word beats, every loss an overrun the
  ring reports, never a corrupt byte (design/dma.md's three copies on
  this arbiter, now with a wire as the sender).
- **Half duplex against a second board** (`9`): this end's HDSEL single
  wire on PA9 (the far PA10) hears the far engine drive it, eight frames
  whole at 9600, 115200 and 1 Mbaud; one open-drain wire between two
  chips - the far G0 swapped onto its PA10 - every frame out heard back as
  itself and answered by the far end at 9600, 115200 and 460800; and the
  wired AND, the far end's 0x00 pulled into this end's 0xFF behind a
  clean trigger frame.
- **LIN both ways** (`0`): the far engine's breaks at 19200 - 9, 10, 11
  and 13 bits with LBDL = 0, 10, 11 and 13 with LBDL = 1 - raise LBD
  from ten and from eleven, never one bit short; and WITH LIN MODE OFF
  the same breaks raise LBD from ten bits on (9: none, 10, 11, 13 and
  20: set, each also a 0x00 frame with FE) - letter `4`'s finding
  staged: 25.4.8 says that with LINEN = 0 the receiver works "without
  taking into account the break detection", and this detector runs
  whatever LINEN says, at LBDL's length. THIS END'S BREAK timed by the
  far receiver, from the break to a 0x55 written as SBK clears:
  13.99..14.00 bit times in LIN mode (thirteen and the one high bit of
  25.4.8) and 10.99..11.06 without (a frame's ten and the bit of 25.4.2),
  over four runs.
- **IrDA SIR against the far ST ENDEC** (`r`), the far G0's TXINV and
  RXINV supplying the one inversion each direction needs (that chip's
  letter `r` reads them on the pad): THIS ENCODER idles LOW with a HIGH
  pulse a zero, 3516 cycles at 9600 and 294 at 115200 (3/16 exactly);
  low power at PSC 49 (1.84 MHz) a pulse of 294 cycles at either rate,
  three periods; the far decoder takes 32 frames of 32 in all four legs.
  THIS DECODER takes the far encoder's 32 of 32 - normal pulses at 9600
  and 115200, low-power ones (1.69 us) at 115200 and at 9600, where the
  pulse is a sixtieth of a bit, in normal mode and in low-power mode
  alike. The turnaround: the far sender's first frame waits 20 ms past
  the settle; without the wait the first start pulse was missed one leg
  in a few (the first byte read one bit late) - the reading is the far
  half-duplex ENDEC, its receiver still taking this end's UART idle,
  behind its RXINV, for a standing pulse. `irda()` refuses normal mode
  with any prescaler but 1 (25.6.7), and the far chip refuses SIR above
  115200, its chapter's ceiling. This USART has no driver enable (25.3,
  table 148): the pair's RS-485 letter is the far G0's, this board's
  PB13 left an input under its DE.

- **Mute mode from a foreign sender, and ES0298 2.13.3** (`s`): THE
  IDLE-LINE WAKE - RWU set after a burst's first frame drops the rest of
  it, the idle frame wakes the receiver with no IDLE flag, the next two
  bursts whole; RWU written BEFORE any frame does not hold (25.6.4's
  note: on an idle line the bit reads back clear) and all three bursts
  arrive. THE ADDRESS MARK on the far `send_mpcm`, this USART's one width
  of FOUR bits: on an 8-bit word (the mark bit 7) address 5 takes 0x85
  and 0x95 - which the far G0's seven-bit mode tells apart - and on a
  9-bit word (the mark bit 8) every group whose low nibble is 5. ES0298
  2.13.3 STAGED - 8O1 frames into this 8E1 receiver, every one a parity
  error, the control with no mute mode flagging all nine: muted by
  SOFTWARE (RWU written), the first matching address wakes the receiver
  with NO PE, its data and the next address with PE - REPRODUCED; muted
  by the HARDWARE (a non-matching address set RWU), the matching
  address after it wakes the receiver WITH PE, every address of the leg
  flagged, three runs alike; awake under the address mark, the matching
  address is checked as any frame. So the erratum - and 25.4.7's own
  note, "the receiver does not check the parity of the address data" -
  is the first address after a software mute; the far G0 flags it in
  every case.

### The rendezvous's bound

Letter `1`'s three runs are this crystal against the far chip's tracked
core three times over, and two crystals move apart by well under a ppm
in the second the runs take: fitted through EVERY stamp of a run
against the far tracker's captures, the ratio read -7.68..-8.13 ppm over
25 runs of 40000 frames and -6.95..-8.85 over 90 runs of 4000, the
fit's own noise in both. So two figures differ by the far measurement's
error alone, and that error is a TIME at each end of the far span,
whatever the span's length - counted and measured with a scratch build
of the far peer that kept every stamp and the tracker's ring behind it:

| the term, per figure | its time | from |
|---|---|---|
| the far stamps: each frame seen at the next turn of a 20-cycle poll, behind the receiver's 1/16-bit start sampling (4 cycles at 1 Mbaud) and up to 5 cycles of DMA on the bus | +-0.45 us (29 cycles of 64.2 MHz) | measured: the stamps' differences on a 20-cycle grid with 3- and 5-cycle extras, the two ends' delays within 26 cycles of each other over 115 windows |
| the far tracker's conversion: exact for its window's mean, the window starting 0.5 to 0.8 ms ahead of the span and stopping up to an interval short of it, the HSI16 wandering hundreds of ppm over such slivers; with the captures' 8-cycle grain | +-0.47 us | measured: -11.6..+8.0 ppm at 40 ms over 90 windows ([../stm32g0/clock.md](../stm32g0/clock.md), "The core against the LSE, tracked") |
| one SysTick handler on a stamp | +-0.86 us | some 55 cycles: the M0+'s entry, the handler's twelve instructions, the return; never seen in 165 windows (one stamp in some six hundred would be) |

Two figures part by at most twice the sum, 3.56 us, and the report's
whole ppm adds one: over 0.4 s that is **10 ppm**, the verdict's bound,
computed from those three times in the suite (`agree_ppm`). Over the 40
ms of a 4000-frame run the same 3.56 us would be 89 ppm, and measured
90 figures of 4000 frames spread -16..+1 (one run of three in the desk's
history at -5/-4/+21): no bound at that length tells two crystals from
an uncorrected RC, whose HSI16 moves between two runs a median 74 ppm
(18 % of 60 pairs under 30, measured over the same windows). At 0.4 s
the HSI moved a median 62 ppm between runs and 8 % of 50 pairs under 10
- a peer that stopped tracking would pass the 10 ppm bound on both
pairs of a run about one time in a hundred and fifty. The three runs
cost 1.2 s.

## The peer: `usart_peer` (the instrument a far board's suite drives)

This stratum's port of the protocol's instrument
(`stm32f4/src/apps/usart_peer.cpp`, firmware 0x0401, the F446RE): one
polled loop on USART1's PA9/PA10 (APB2, 90 MHz), dark until a frame
checks out, a bounded action after every ack. The engine is this
chapter's:

| the chapter's offer | in the peer |
|---|---|
| one data register and the shifter | the echo and the sink polled a character at a time: SR read once a turn, then DR, THEN the echo's DR write - ES0298 2.13.2's own order (a DR write between the SR and the DR read clears PE in full duplex) |
| transmit and receive DMA | DECLINED: the polled loop costs a fraction of a frame (below), and a blast is a TXE poll and a store, gapless at 6.4 Mbaud |
| OVER8 | USED where it is closer to the rate (eighths: 90 MHz / k, k >= 8), and alone under the protocol's clk2x flag; 11.25 Mbaud is the ceiling |
| TIM1_CH2 / TIM1_CH3 on PA9 / PA10 (AF1) | USED: THE WAVEFORM ENGINE - the stm32g0 peer's shape on this chip's parts: TIM1 at 180 MHz (APB2's timer clock), DMA2 stream 6 channel 0 (TIM1_CH1/2/3 share it, table 29, one CCxDE at a time), direct mode on half-words, the absolute compares, the pad sampled and the run timed whole; the `stage` op its widest user - a train of frames drawn cell by cell, a pulse inside each, the gaps in sixteenths of a bit |
| HDSEL | USED for the shared wire: the single wire on PA9 (the far G0 swaps its own onto the same wire), open drain on its pull-up |
| the four stop codes | served on this FULL instance (25.4.3 recommends the halves for smartcard mode and forbids them nowhere else) |
| eight- and nine-bit words | 7+P, 8, 8+P and 9 served; 5, 6 and 7 data bits without parity, 6+P and 9+P REFUSED |
| synchronous master | REFUSED: no slave on this USART (25.4.9) and the CK pad, PA8, reaches the far board only through a strap the desk has not got |
| IrDA SIR | IREN to 115200; normal mode at PSC = 1, the one value 25.6.7 allows there, so a receive filter is REFUSED in it; the low-power pulse from the requested width; no line inversion on this USART - the far G0 supplies both |
| LIN, mute, smartcard | the DUT's subjects |

The time base is the crystal: the ST-LINK's 8 MHz in bypass through the
PLL, a DWT cycle and a TIM1 count each 2/15 of a 24 MHz tick, and a
`send` or `blast` states the rate its divisor gives on it (the Report's
aux16 with `report_rate_stated`: 90 MHz over BRR, or over 8 x mantissa
+ fraction at OVER8, 0 where that is exact). On
the shared wire THE REPLY'S OWN ECHO LANDS AFTER TC: the HDSEL receiver
raises RXNE for the frame it just sent a fraction of a bit after the
transmitter's TC, so a flush at TC left it standing and the loop
answered its own echo inside the DUT's next frame (measured: the DUT's
frames corrupted from their fifth bit, two of every three); the peer
now waits for that echo, two bit times at most, before it flushes. **The
cost**: the echo's clean character is 23 instructions with no call
(release listing) - the SR load and its tests, the DWT stamp, the DR
load, the DR store and the count - some 35 cycles against the 281 of a
6.4 Mbaud frame. The engine's floor: an interval of 8 counts (44 ns)
played, of 6 the compare was written after the counter passed it; 12 is
the floor the peer keeps.

## Not covered yet

Driver gaps:
- Nine-bit words through the TASK (the resource's `write_word`/
  `read_word` speak them; the rings carry bytes) - declined, as on the
  other strata.
- A wake from Stop: this USART has no wake unit (the G0's WUF); the
  power chapter says what a Stop does to a pending receive.
- The receive errors' COUNTS under the engine are bounded, not exact
  (above): exact counts would need the CPU's read of DR, which the
  engine's rule forbids - declined.

Implemented, not bench-verified (every mode of the resource beyond the
transport - each waits for the wire or the peer that measures it): the
smartcard (no card on the
desk), the synchronous clock (a timer capture on CK - the timer
chapter's ruler), CTS/RTS flow control (a peer that asserts them), the
TC/CTS/LBD interrupt enables, `set_baud` at run time (nothing changes the
LINK's rate with the clock standing still), the instances beyond USART1,
USART2 and USART6 driven on the F446 and the consoles' USART1 and USART3
(UART4/5/7/8 compiled on every header that has them, none driven), the
OVER8 and frame-format legs on the other three boards (the loop's pad is
surveyed on the Nucleo-F446RE alone).

Implemented, not bench-verified, in the peer: the half stop codes as a
SENDER, which no letter judges (its `send_mpcm`, `stage`, `autobaud_tx`,
`brk` and IrDA are the far G0 suite's letters `s`, `5` and `9`, `0` and
`r`).
