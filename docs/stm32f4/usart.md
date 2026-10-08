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
(`stm32f4/device_tables.hpp`), with the engine slots filled from
[dma.md](dma.md) (`stm32f4/dma_engine.hpp` the empty slot's tag). The
family fixture is `test/family_stm32f4/usart.cpp` with the negatives
that refuse an absent instance, flow control on a UART, an engine off
the instance's cells of the request mapping, an engine on a part class
whose manual was not read, a receive ring longer than one lap of the
stream's count, and one pad twice. Bench: the console apps
and the platform suite on the four boards, the engined console in
`test_stm32f4_dma`, the transport on USART6's single-wire loop in
`test_stm32f4_serial` (the Nucleo-F446RE), its cost in `bench_stm32f4`
letter u.

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
  half_duplex, rts, cts, rts_pin, cts_pin, tx_speed,
  single_wire_push_pull}` - the defaults are the console's. In half
  duplex the TX pad is open drain on its pull-up unless
  `single_wire_push_pull`, which drives it push-pull while a frame goes
  out (the transmitter releases the pad between frames, 25.4.10, and the
  pull-up holds it): for a line with no other driver, because the open
  drain rises on the pull-up alone - 1 Mbaud carried, 2.8 not, against
  11.25 push-pull (letter a).
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
  `frame_errors`, `parity_errors`, `noise_errors`, `hw_overruns`,
  `clear_errors`, `rebase(hz)` (the ClockUser verb: `hz` is SYSCLK and
  the bus rate is DERIVED from it with `apb_hz_at`, not read back from
  the RCC, because a dynamic clock fans the new rate out BEFORE the
  prescalers move), `set_baud(hz, baud)`, `divisor_for`,
  `min_hz_for`, `can_baud`, `actual_baud(fck)`, `kernel_hz<Clock>()`,
  `release()`; with engines, `dma_isr()` (the streams' vectors' body,
  whose true is the receive edge as `isr()`'s), `harvest()` (the same
  edge asked from the consumer's side, below) and `dma_faults()`. Refused at compile time: invalid or coincident pads,
  an engine off the instance's request cells, flow control on a UART, a
  flow pad missing, and with a receive engine a ring above 32768 bytes.
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
(OVER8 with ONEBIT, APB2 / 8 - BRR 0x20); every frame format of the task
- 8N1, 8E1, 8O1, 7E1, 7O1, 8N2 - byte-exact at 1 Mbaud, a seven-bit
frame's eighth bit being its parity bit as the stream stores it (DR's
MSB, 25.6.2; the interrupt receiver hands it on too); the open drain on
the internal pull-up whole at 1 Mbaud and 31 of 64 at 2.8125.

**Errors under the engine** (letter d): 120 data bytes with 14 breaks
between them, each followed by a clean frame - all 120 delivered intact
and in order, the 14 breaks stored as the 0x00 frames they are, FE
counted 14: no byte taken by a clear. Two breaks back to back count 1,
three count 2. Ten breaks under a thread polling SR as fast as it can:
7 counted. The interrupt receiver drops each break's frame and counts
it (40 of 40 data bytes, FE 10 for 10).

**`tx_idle()` is the wire's** (letter e): its first true lands 1570 and
1579 cycles after the last stop bit's rising edge on the pad at 115200
(a bit is 1562) - interrupt transmitter and transmit engine - and 202 and
203 at 1 Mbaud (a bit is 180), the edge's EXTI latency taken off: within
the few cycles of TC rising at the stop bit's end, on both paths. Before
this round the verb was the ring's, a frame early.

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
transport - each waits for the wire or the peer that measures it): mute
mode with both wakes, LIN's break and detection, single-wire half duplex
against a second board, IrDA (an IR pair), the smartcard (no card on the
desk), the synchronous clock (a timer capture on CK - the timer
chapter's ruler), CTS/RTS flow control (a peer that asserts them), the
TC/CTS/LBD interrupt enables, `set_baud` at run time (nothing changes the
LINK's rate with the clock standing still), the instances beyond USART2
and USART6 driven on the F446 and the consoles' USART1 and USART3
(UART4/5/7/8 compiled on every header that has them, none driven), the
OVER8 and frame-format legs on the other three boards (the loop's pad is
surveyed on the Nucleo-F446RE alone).
