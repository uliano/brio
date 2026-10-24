# USART (CH32V203, CH32V303)

This document is for a programmer who needs a serial port on a CH32V203
or a CH32V303: the console a program prints to, a line-oriented link, a
byte transport to another chip, or one of the modes the peripheral
offers (mute mode, LIN's break, IrDA, the smartcard, a synchronous
clock, half duplex on one wire, the flow-control pair). It covers the
serial ports of the reference manual's chapter 18 - USART1..USART3 and
UART4..UART8, as many of them as the part offers - in the two layers
every brio serial driver has ([design/serial.md](../design/serial.md)):
`Usart<n>`, the resource, a monostate (a class with only static
members, never instantiated: one peripheral, one set of state) that
knows which instance it is, where its registers are, its bus gate and
its reset, its vector, its DMA slots, and the chapter's register
description as verbs; and `Uart<n, P, ...>`, the task (a task, in brio,
is a driver layer with state of its own, served from its peripheral's
interrupt, that an active object or `print` calls; an active object is
a component of a brio program with its own event queue, served one
event at a time by the kernel,
[docs/learn/introduction.md](../learn/introduction.md)): the
interrupt-driven byte transport every console runs on, with a transmit
ring and a receive ring, an optional DMA engine in either slot, and the
public surface `util/line_reader.hpp` and `print()` are written
against, which is what lets them run here unchanged.

The smallest use, the console on USART1's default pads PA9/PA10, which
both boards wire to the probe's own serial ([README.md](README.md),
"Serial console") - the first block under "How to use it", whole:

```cpp
using Serial = brio::Uart<1, P>;                 // USART1, PA9/PA10, 8N1
constexpr Serial serial;
Serial::init(clock, 115200);
BRIO_CH32_VECTOR(usart1_handler) { (void)Serial::isr(); }
```

Line by line: `P` is the platform, `brio::Ch32vx03Platform<>`
([platform.md](platform.md)), which the rings ask whether an index can
be shared with the interrupt handler bare or wants a guard (the
platform's `atomic_width`). `Serial` names the transport type: `1` the
instance, and every other template argument at its default - a receive
ring and a transmit ring of 64 bytes, the 8N1 frame, no DMA engine in
either slot, remap code 0 (the reset column, PA9/PA10 for USART1, for
which `init()` writes no AFIO register at all) and the default
`UartOptions`. `serial` is an empty tag object of that type, the
argument `brio::print(serial, ...)` takes to pick the port
([util/print.hpp](../../brio/util/print.hpp)), since the type's
functions are all static and it has no object of its own.
`Serial::init()` runs in `main()`, after the clock task's
`SysClock::init()` and before `brio::enable_interrupts()` and the
kernel's `run()`. It takes the clock task's tag (`constexpr SysClock
clock;`, declared as in [clock.md](clock.md)'s smallest use) and the
baud rate; it computes the divisor from the rate of USART1's own bus,
PCLK2 (a BRR of 1250 for 115200 under a 144 MHz clock), opens the bus
gate, hands TX to the peripheral as an alternate-function push-pull
output and RX as a floating input, writes the frame and the divisor,
enables the port with its receive interrupt armed, and enables USART1's
line in the PFIC. It answers false, before it has written any register
of the port, for a divisor outside 16..65535 (1200 baud from the 144 MHz
bus, "The frame and the divisor" below) and for a column on the debug
port's pads while the probe owns them (that check reads AFIO's PCFR1,
which opens AFIO's gate first); and, with a receive engine, for a ring
its channel refuses. The last line is a definition at file scope,
outside any function: `BRIO_CH32_VECTOR(name) { body }`, or one of its
two forms, is the one spelling of an interrupt handler on this target
([platform.md](platform.md)), `usart1_handler` is the name the crt (the
startup code that runs before `main()`,
[startup_ch32vx03.S](../../ch32vx03/src/glue/startup_ch32vx03.S)) gives
USART1's vector, and `isr()` is the port's whole interrupt body: it
takes each received byte into the receive ring and feeds the data
register from the transmit ring, and returns true when a received byte
made the receive ring non-empty, the edge an active object is told of.
The `(void)` discards that answer; a program whose line-assembling
active object (a `LineReader` of `util/line_reader.hpp`) waits for the
edge posts `brio::RxActivity{}` to it on true, as the fourth block under
"How to use it" does. `print` is then called after
`brio::enable_interrupts()`, from `main()` or from an active object's
event handler.

What decides correctness even there:

- **The clock task first.** `init()` computes the divisor from the rate
  the clock type claims for the instance's bus, not from the hardware:
  USART1 counts PCLK2, the clock of the peripheral bus PB2, which a 144
  MHz clock type claims at 144 MHz, and every other port counts PCLK1,
  the clock of the bus PB1, which this stratum caps at 72 MHz
  ([clock.md](clock.md)). A port initialized while the chip still runs
  on its 8 MHz HSI reset rate, every bus at 8 MHz, speaks at the ratio
  of the two rates until the clock task has run - USART1 at an
  eighteenth of the rate asked (6400 baud for 115200), a port on the PB1
  bus at a ninth (72 MHz claimed, PB1 halved under the 144 MHz type,
  against 8) - and at the rate asked afterwards, the divisor being
  written once (derived from the divisor arithmetic, not measured).
  Nothing refuses it: `init()` judges the divisor's range against the
  claimed rate alone. What the compiler does check is that a dynamic
  clock lists the port among the users it rebases (`init()`'s
  `clock_follows` assertion, [clock.md](clock.md)).
- **The vector bound by the program.** `init()` arms the receive
  interrupt and enables the line, and the first byte written arms the
  transmit one. Every handler the crt's vector table names is a weak
  alias of `default_handler`, a spin loop, until the program defines
  it, so a port whose vector is left unbound stops the program in that
  spin at its first received byte or its first `print` once interrupts
  are enabled, with no fault and no message (the probe names the line:
  [README.md](README.md), "Debugging"). On the CH32V303's images, built
  with the F extension and with the hardware prologue on (the project's
  `CH32VX03_HPE` option, a CMake option of the `ch32vx03/` project
  chosen at configure, on by default), `BRIO_CH32_VECTOR` makes a naked
  trampoline whose body must do no floating-point work; `isr()` does
  none, and `brio check ch32vx03` fails an image whose trampoline body
  reaches a floating-point instruction ([platform.md](platform.md)).
- **An instance the part offers.** `Uart<n, P>` of an instance the part
  has not got does not compile. The CH32V203F6, the smallest part,
  offers USART2 alone - its package bonds neither of USART1's pin pairs
  (the part table,
  [parts/ch32v203f6.hpp](../../brio/ch32vx03/parts/ch32v203f6.hpp), the
  per-part facts `device::` exposes) - so `Uart<1, P>` is a compile
  error there ("Up to eight instances on two buses" below).
- **`print` after `init()` and with interrupts enabled**, never before
  ([util/print.hpp](../../brio/util/print.hpp)'s rule). `print` waits
  until the transmit ring has taken every byte, and only the transmit
  interrupt drains the ring: printed before `brio::enable_interrupts()`,
  or into a port whose `init()` answered false, a text longer than the
  ring holds (63 bytes of the default 64, derived from
  [util/ring.hpp](../../brio/util/ring.hpp)'s capacity) spins for ever,
  and what a shorter one does there is promised by nothing.

Three more things to know before the first `print`: a long message
printed from an active object's event handler stalls every other active
object for as long as `print` waits
([design/serial.md](../design/serial.md)); the transmit ring has one
producer, so `write_byte` and `write_bulk` are called from active
objects and from `main()`, never from an interrupt handler, and both
copy, the caller's buffer being free when the call returns; and a DMA
engine in either slot keeps the core awake while its channel is
enabled, because in a sleep of any depth on this family no bus master
but the core gets a cycle (the count of active bus masters,
[sleep.md](sleep.md)) - a receive engine's channel runs, circular, from
`init()` to `release()`, and the program does not sleep while it does.

The page is long because the chapter is. In order: the documents of
record; what the silicon does (the instances on two buses, the frame
and the divisor, the modes, the receive side's offer and what the
transport takes of it, the DMA requests as slots with the receive
engine's ring, the CH32V303's lot-keyed features, what a second chip
measures over the desk pair's link, and this part as the desk's
instrument); the types and verbs; how to use it, one block per use; the
bench findings; what is not covered.

The page's subject, in full: the serial ports of RM chapter 18 - the
asynchronous frame in every shape the register description allows, the
fractional baud generator, mute mode with both wakes, LIN's break,
single-wire half duplex, IrDA, the smartcard, the synchronous clock,
the flow-control pair, two DMA requests, every flag and both interrupt
sources, and on the CH32V303 the MARK and SPACE parity and the short
words of a lot - and the interrupt-driven byte transport every brio
console is, sitting on that resource with its options costing nothing;
and the desk pair's serial link, the CH32V203C8's USART3 crossed to
UART4 on the CH32V303's second column, each chip the other's device
under test (DUT) and, through its usart_peer (the peer firmware the
suite's two-board key talks to, "This part as the desk's instrument:
usart_peer" below), the other's instrument.

## Documents of record

The CH32FV2x_V3xRM reference manual V2.3 (the chapter's opening for
which ports are USARTs, 18.2 for the frame and TE's idle frame, 18.3
for the baud generator, 18.4 for the synchronous mode, 18.5 for half
duplex, 18.6 for the smartcard, 18.7 for IrDA, 18.8 for DMA, 18.9 for
the interrupts, 18.10 for the registers with 18.10.1, 18.10.4 and
18.10.8's notes for the lot's four features; 3.4.7 and 3.4.8 for the
gates and the two peripheral buses; 11.2.3's tables 11-2 to 11-5 for the
DMA slots; 9.5.1 and table 9-2 for the vectors; 15.3.10 and 15.4.7 for
the one pulse and the output modes of the peer's waveform engine) with
its class rule - three classes read the chapter here: CH32V20x_D6 for
every CH32V203 up to the CH32V203C8, CH32V20x_D8 for the CH32V203RB and
CH32V30x_D8 for the four CH32V303 - the CH32V203 datasheet V2.8 (table
2-1 for how many serial ports a part has, the pin tables of 3.2 for
which) and the CH32V303 datasheet V3.5 (table 2-1-1, the pin tables of
3.2 and the alternate functions of 3.3), AFIO's tables 10-23 to 10-31
(the columns, [pin.md](pin.md)), the driver
[brio/ch32vx03/usart.hpp](../../brio/ch32vx03/usart.hpp) and the
reference suite `test_vx03_serial` with its two-board key (a suite's
letter is one command key of its console and the group of verdicts it
runs; `z` is the key that runs every single-board letter, and `y` the
two-board key, the letters run against the far chip's usart_peer - the
digits 1 to 9 and, on the CH32V303, n and o).

## What the silicon does

### Up to eight instances on two buses

- **Which ports a part has is the datasheet's table.** USART1 answers on
  PB2 and counts PCLK2; every other port answers on PB1 and counts
  PCLK1, which this stratum caps at 72 MHz ([clock.md](clock.md)) - so at
  a system clock of 144 MHz the two buses divide different numbers and
  every divisor is asked of the instance's own. The packages below the
  CH32V203C8 count two serial ports and the smallest counts one, and it
  is not USART1; the CH32V203C8 and RB count four; the 128 KB CH32V303
  three (USART1..USART3); the CH32V303RC and VC all eight, USART1..USART3
  and UART4..UART8. `device::has_usart(n)` is a mask, not a count.
- **The upper ports are not at the addresses their numbers suggest**:
  UART5 follows UART4 at 0x4000 5000, while UART6, UART7 and UART8 sit
  below USART2, at 0x4000 1800, 0x4000 1C00 and 0x4000 2000 (tables 18-6
  to 18-9); their gates are PB1PCENR's bits 20 and 6..8 (3.4.8).
- **The fourth port is a USART on the CH32V203C8 alone.** Chapter 18's
  opening counts three USARTs and five UARTs for the whole family and
  then names the exception - "for CH32V203C8 the serial port 4 is a
  universal synchronous asynchronous receiver transmitter" - and that
  part's pin table agrees, bringing out USART4_CK, USART4_CTS and
  USART4_RTS. On the CH32V203RB and on every CH32V303 it is a UART4 with
  TX and RX alone, and UART5..UART8 are UARTs everywhere. So the
  synchronous clock, the smartcard and the flow-control pair are a part
  fact (`device::usart_full`), and the verbs that configure them are
  refused at compile time on an instance that is not full; the questions
  about them (`smartcard_enabled()`, `synchronous_enabled()`) answer
  false there. Measured on the CH32V203C8T6: the fourth port's CK pad
  carries a clock, and its vector is the one at that class's own tail.
  Measured on the CH32V303VCT6: UART5..UART8 each opened by its gate,
  its divisor read back, a byte read off its transmit pad by software
  and a frame banged into its receive pad through the pad's own pull,
  byte for byte, with no wire on any of them.
- **The pads are AFIO's columns** (tables 10-23 to 10-31), which the
  driver takes from the same table that programs the register - TX, RX,
  CK, CTS and RTS move together. UART4 reads a different table per
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
register file on both series, and the bench suite's single-board letters
a..l and r run on both parts.

- **The bit is the divisor.** BRR counts peripheral clocks per
  sixteenth of a bit, so the whole register is pclk/baud and a start bit
  lasts exactly BRR peripheral clocks: measured to the ruler's own count
  at 9600, 115200, 921600 and 3 Mbaud. Every standard rate from 1200 to
  3 Mbaud comes out within one per mille of what was asked on both buses.
- **The divisor's range refuses at the slow end, not the fast one.**
  Sixteen is the floor (one whole clock per bit sixteenth) and 65535 the
  register: from a 144 MHz bus, 1200 baud asks for 120000 and is refused,
  while every rate up to 3 Mbaud is far above the floor. `init()`
  answers false rather than mangling every frame.
- **The frame is a word length plus a parity choice**: M gives 8 or 9
  bits including the parity bit, so seven data bits exist only with
  parity and nine only without. Measured on the wire as the low run of a
  zero word: nine bit times for 8N1 and 7E1, ten for 8E1 and for a
  nine-bit word.
- **Two of the four stop codes are an ordinary frame's.** The gap
  between two zero words sent back to back is one bit time under the 1
  code and two under the 2 code - and also one and two under the 0.5 and
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
  add up raises PE and is still delivered - the receiver reports, it does
  not drop - and a low stop bit raises FE. On an overrun the data
  register's byte survives and the shift register's is lost (measured:
  the first of two frames is what comes out), and the read sequence
  clears ORE.
- **IDLE rises once** after a frame followed by a line that stays high,
  and not again until RXNE has been set (18.10.1's note, measured).
- **A read of STATR arms the clear; the next read of DATAR performs it,
  on every flag standing at that read** - measured on the CH32V203C8T6
  with the receive channel reading DATAR and the CPU never: a break's FE
  raised after a status read is gone once the channel has read the
  break's own frame, and one raised with no status read since the last
  DATAR read stands (FE and IDLE both read 1 three milliseconds later);
  and clearing IDLE that way also forgets the idle the clearing frame
  armed, so one frame after an idle that was read raises no IDLE of its
  own and two frames do. The STM32F4's block, under the same words in
  its manual, clears only what the status read saw
  ([../stm32f4/usart.md](../stm32f4/usart.md)) - a difference of the
  silicon the transport's receive engine is written from (below).

### The modes

- **Mute mode**: with RWU set, frames are not received; under WAKE 0 an
  idle line clears RWU by itself and the next frame is taken; under
  WAKE 1 a nine-bit frame whose MSB is set and whose low four bits are
  ADD wakes the receiver, and another node's address does not.
  18.10.4's two notes are real: a byte must have been received before an
  idle-line wake works, and RWU cannot be set under an address mark
  while RXNE stands (`mute()` refuses).
- **The break** SBK sends is ten bits of low outside LIN mode and
  thirteen inside it (measured on the transmit pad: 150000 and 195000
  timer clocks at 9600, where a bit is 15000). The detector raises LBD
  once at ten low bits and not at nine; with LBDL set it wants eleven and
  ten are a frame; LBDIE carries it to the vector.
- **IrDA's two directions do not share a level logic** (18.7 says so,
  and here is what it means): the encoder puts a high pulse of three
  sixteenths of a bit on an idle-low pad for every zero (measured 2814
  timer clocks for the 2812 of 3/16 at 9600), or three periods of the
  prescaled clock in low-power mode (1200 for 1200 with a prescaler of
  200); the decoder takes the mirror - an idle-high line whose zeros are
  low pulses. A frame banged the encoder's way reads 0xFF, the same
  frame banged the other way reads its byte.
- **The synchronous mode is the master's alone** (18.4): the column's CK
  pad carries one pulse per data bit while the transmitter shifts and at
  no other time, and CPOL is the pad's idle level (measured both ways).
  **LBCL reads as the F1 family's and not as 18.10.5's words**: the
  manual says the pulse of the last data bit is output when the bit is
  clear, and the silicon does the opposite - eight bytes give 112
  transitions (seven pulses each) with LBCL clear and 128 (eight) with
  it set. CPHA is the receiver's capture edge and moves nothing on the
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
  On the pad it is the peripheral clock divided by twice the prescaler
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

### The receive side's offer, and what the transport takes of it

| the chapter's offer | taken? | why |
|---|---|---|
| DATAR, one level: no FIFO | the interrupt receiver takes every byte at RXNE, one entry a byte | nothing deeper to batch; 4.5 Mbaud on the crossed pair carried byte for byte |
| DMAR and a circular channel ([dma.md](dma.md)) | the receive engine | the bulk path: no entry a byte, no re-arm |
| IDLE and IDLEIE (18.10.1) | the engine's burst edge | one frame after the last stop bit; the channel's read finishes its clear, so the vector waits for the next frame with IDLEIE disarmed |
| RXNEIE beside DMAR | the engine's wait for a frame | the channel still takes the byte and the vector learns one came (measured: one entry a frame, RXNE already clear, CNTR moved) - the edge of a burst of one frame; ORE raises it too, which is how an overrun with nothing left to take is found and cleared |
| EIE (FE, NE, ORE under DMAR) and PEIE | counted by the engine's vector | an error is counted when it rises and disarmed until the channel's next read clears it |
| the half and full marks of the channel's lap | the engine's edge with no silence | a stream that never pauses is told twice a lap |
| a receiver time-out | none on this block | |
| the LIN break flag | not taken | the transport carries no LIN; a break reaches the receiver as the 0x00 frame with FE it is |
| TC written 0 (18.10.1) | at every transmit block's start | the channel's writes of DATAR run no part of TC's software clear; `tx_idle()` must not answer from the frame before the block |

### The DMA requests are slots

On this family the channel is the request ([dma.md](dma.md)), and on the
CH32V303 a request lands on one of two controllers, so each direction of
a port answers a slot - the controller and the channel. USART1 transmits
on DMA1's channel 4 and receives on 5, USART2 on 7 and 6, USART3 on 2
and 3, on every part that has them. UART4 is DMA1's 1 and 8 on the
CH32V203 (tables 11-5 and 11-6) and DMA2's 5 and 3 on the CH32V303
(table 11-3), whose UART5..UART8 are DMA2's too: 4 and 2, 6 and 7, 8 and
9, 10 and 11 (tables 11-3 and 11-4). An engine on any other slot is
refused at compile time. A block start holds the transport's guard over
the transmit engine's claim alone - the busy flag's test-and-set, eight
instructions - and programs the claimed channel, four stores, unmasked:
no block is in flight that could complete under it ([dma.md](dma.md)).

`release()` pulses the block's reset before it gates the clock: a
transmit request is raised as soon as DMAT meets an idle transmitter -
at `init()` already, with an engine - and is held through DMAT, DMAR and
UE cleared. Left standing, it would be served by the next owner of
USART2's channels (I2C1's, TIM4's, TIM2's) as one stray item, or stall
it ([dma.md](dma.md), "A released requester"); the pulse is two
read-modify-writes of the bus's reset register, at release only.

The receive engine runs a ring, not runs. It is armed in its circular
shape over the whole receive ring's storage, from `init()` to
`release()`: the channel writes the storage lap after lap and is never
re-armed, the ring's producer index is the channel's own count, and the
transport's receive ring is the consumer half of that -
[util/ring.hpp](../../brio/util/ring.hpp)'s `HardwareRing` over a
storage the transport owns, its size a power of two up to 32 KB, aligned
to its own size where the slot is the CH32V303's DMA1 so the 64 KB rule
cannot refuse it. A byte that lands is readable at once, a burst wraps
the storage's end with no CPU, and nothing is lost between runs because
there are none - measured with bursts banged into the receive pad
through its pull ([dma.md](dma.md)). What the consumer did not keep
up with - a lap written over its unread bytes, or a run written over
while it was held - the ring counts and skips, and `rx_overruns()`
reports it; `consume()` answers whether the run it releases was intact.
With a receive engine the handler never reads DATAR - RXNE is the
channel's request, and a read would take a byte from under the channel -
and its receive half is the two states of `usart.hpp`'s header, written
from the clear measured above: waiting for the end (IDLEIE, PEIE, EIE),
where an idle line or an error is counted, its clear armed by that status
read and the edge reported, then waiting for a frame (RXNEIE alone),
whose entry reads no STATR - a status read there would arm the clear the
next frame's own read performs, and that frame's error would vanish
unseen - but CNTR: a moved count turns the vector back and reports the
edge again, which is the only edge a burst of one frame gets. A port
with no transmit engine serves its interrupt transmitter from the same
vector, so that entry asks CTLR1 whether TXEIE stands and reads STATR for
TXE only then - the interrupt transmitter's entry is one of the status
reads that arm the clear (above). A frame
the channel never takes - a channel stopped by a transfer error, or one
whose handshake another peripheral's held request froze - leaves RXNE
standing under RXNEIE, a level that would re-enter the vector for ever
with the thread starved: the entries of that wait that find the count
unmoved (and, with no transmit engine, TXEIE down, so that they are not
the transmitter's) are counted, and the 255th gives the channel up -
`dma_faults()` counted, RXNEIE down, the edge reported, nothing else
touched in the vector - and the consumer's next look stops the channel
and starts the ring again from its first element, as after a transfer
error, and re-arms the wait for the end. A live channel takes the frame in one or
two entries; a channel a memory-to-memory block outranks and starves for
255 of them - a ring a program armed below its copy - is given up too.
An overrun with nothing to take is the other way that wait re-enters
without end: a channel starved past a frame time leaves ORE up, takes
the frame DATAR held when it is let go, and with no frame after it - a
message's tail - RXNE is down, the channel has nothing to move, and
RXNEIE, which ORE raises on its own (18.10.4), re-enters the vector back
to back until the bound gives a live channel up; the restart's EIE then
re-enters on the same ORE, counts it a second time, and the next storm
begins. So the wait's second entry with the count unmoved reads STATR
once and, finding ORE without RXNE, reads DATAR - 18.10.1's clear - and
turns back to the wait for the end. That costs the counts nothing:
with the count unmoved no DATAR read has come since the status read that
began the wait, so the clear that read armed is armed still, a second
status read arms nothing new, and the DATAR read performs it over flags
that read already counted - the overrun among them, counted once - and
an IDLE risen since, whose edge the entry reports. A frame landing
between the two loads keeps its byte - the request it raised outlives
the CPU's read and the channel moves DATAR's frame when it is served
(measured below) - and loses its error flags to the clear, as the frame
after any counted error does. The channel's half and full marks report
it from `dma_isr()`. Two interrupts
a burst, none a byte. What the channel's read bounds is the counts and
never the bytes: the frame after one whose error was counted loses its
own (a run of errored frames counts every other one), so does the first
frame of a burst after an idle the vector saw, and a status read anywhere
else - `tx_idle()`, the interrupt transmitter's entry, a thread polling a
flag - arms the clear for the next frame's error.

The ring outranks the copies. DATAR holds one frame, so a receive
channel starved for a frame time loses the next one: the receive engine
arms at the controller's highest level, very_high, and the transmit
engine at high - a transmit channel that waits leaves the line idle and
loses nothing - the rule of [../design/dma.md](../design/dma.md).
`UartOptions::rx_priority` is the one knob, for a program that ranks
another channel above its ring; a port with no receive engine refuses
it at compile time.
Measured on the CH32V303VCT6 against the far chip: UART4's two engines on
DMA2, on its second column, a message round the far chip's echo and then
four kilobytes each way at 921600 baud, every byte in order and no error,
overrun or fault counted at either end.

### The lot's four features (the CH32V303's)

Four things the chapter gives the CH32V30x_D8 - and there only "with the
penultimate sixth digit of the lot number not being zero", which no
register states and the part number does not carry - and none of them
the CH32V203's, where every verb that names one is refused at compile
time:

- **CTLR4 (18.10.8)**, a register of its own at offset 0x1C: CHECK_SEL
  holds the frame's parity bit at one (MARK) or at zero (SPACE) instead
  of computing it, and MS_ERRIE is the interrupt of a received parity
  bit that did not hold that level.
- **CTLR1's M_EXT (18.10.4)**: a word of seven, six or five data bits,
  where 00 leaves the word to M.
- **STATR's MS_ERR and RX_BUSY (18.10.1)**: the MARK/SPACE error, read
  and cleared as PE is, and a receiver in the middle of a frame.

A program cannot read its lot, and this stratum's rule for a lot's
register ([README.md](README.md)) is that the verb asks the die. M_EXT is
a field of a register every die has, so `short_word()` writes it and
reads it back, and puts it back to zero where the die did not keep it.
CTLR4 is a register of its own, and an absent register can answer as a
mirror of another ([opa.md](opa.md)'s EXTEN_CTR2), so `ctlr4_present()`
reads the word first - the real register holds nothing outside its three
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
arrived at 1200 baud. And the wire agrees, which is what tells an absent
register from a write-only one: with CTLR4 stored raw, past the verbs, a
zero byte's low run stayed ten bits under even parity with MARK and nine
under odd parity with SPACE - the parity bit computed, as without it -
and with M_EXT's seven- and five-bit codes stored raw a zero word stayed
nine bits low, eight data bits and the start. It is the lot the other
lot-keyed registers of this die already said it is
([README.md](README.md)).

### The pair: what a second chip measures

The desk pair's serial link - the CH32V203C8's USART3 on its default
column (TX PB10, RX PB11) crossed to UART4 on the CH32V303's second column
(TX PB0, RX PB1), both on PCLK1 at 72 MHz, each board on its 8 MHz
crystal - carries the two-board key of the reference suite both ways:
each chip's receiver and transmitter judged by the other chip's, which
neither a loop nor a crossed pair on one board can do (both divide one
bus by one arithmetic, and agree with themselves). The same numbers came
from both directions unless one is named.

- **The two crystals agree to a few ppm.** A gapless run of 4000 frames
  at 1 Mbaud (a divisor of 72, exact at both ends) timed by the far
  receiver read -9 to -3 ppm from the CH32V203C8T6 and +1 to +6 from the
  CH32V303VCT6 over ten runs each way, and the far chip's blast timed here
  the opposite (0 to +5, -6 to -3): one pair of crystals seen from its two
  ends, each span good to 24 ppm (the two polled stamps, about 30 cycles
  each, over 40 ms). Either die on its HSI would read 3000
  ppm and more ([clock.md](clock.md)).
- **Every rate is PCLK1 over BRR on the wire, 9600 baud to 4.5 Mbaud.**
  Sixteen bytes echoed exact at 9600, 115200, 921600, 1, 2, 3 and 3.6
  Mbaud and at 4.5 - 18.3's divisor floor of 16 on a 72 MHz bus, both
  ports' top - and at each a gapless run timed by the far receiver against
  a 1 Mbaud run back to back, ten runs each way: within 33 ppm of the
  divisor's 0 at every exact rung (the stamps' bound 52 to 94 ppm at these
  spans), and +1593 to +1611 at 921600, where BRR 78 gives 923077 baud
  (+1602).
- **The half stop codes are rounded up on the wire**, as the single-board
  measurement above found on the transmitter's own pad, and now by a
  foreign receiver: gapless runs at 1 Mbaud under the four codes, the 0.5
  code's span within 8 ppm of the 1 code's, the 1.5 code's within 3 of
  the 2 code's, and the 2 code 1.1 of the 1 (22 half bits against 20, read
  +99992 to +100006 ppm).
- **The receiver's tolerance is four to five per cent.** 0x00 frames drawn
  by the far engine at 9600 x (1 + e) - the rate its crystal's and not a
  divisor's - into a receiver at 9600: clean from -4 % to +5 %, broken
  from +6 % (NE and wrong data, a fast frame's stop sampled late) and
  from -5 % (FE), on both receivers. 18.3 promises not less than 3 %.
- **The start bit's filter**: a single low pulse of the far timer's one
  pulse, 1250 ticks wide at 9600 (half a bit), is no start bit; 1875
  (three quarters) is one, with the frame it begins; nothing under 300.
- **The sampling, and its rule left open.** One low pulse walked across a
  data bit of 0xFF frames at 9600 finds three sensitive instants a
  sixteenth apart, the first between 37 % and 44 % of the bit, the three
  moving together by up to a sixteenth from one burst to the next (the 16x
  clock runs free of the line, the start edge found on its tick); within
  one train of frames the phase holds. A pulse over all three flips the
  bit with no NE, one over none changes nothing, one over one or two of
  them raises NE - and which way the bit then goes is not the three
  samples' majority, the STM32F1's documented rule, in every case: on the
  CH32V203C8T6's receiver a 1 % pulse over the first or the last instant
  alone flipped the bit (with NE) and one over the middle one kept it,
  wherever the census hit them, and a 14 % pulse over the first one or two
  instants flipped it in seven runs of ten there and in four of ten on the
  CH32V303VCT6's, keeping it in the others.
- **The errors from drawn trains**: a parity cell drawn the other way
  raises PE on every frame with the byte delivered; a stop cell drawn low
  raises FE with the data intact; a zero frame with its stop low is a
  ten-bit break, LBD under LBDL 0 and the 0x00 frame with FE; a run of
  eight read late leaves ORE with RXNE, DATAR keeping the run's first
  frame; and the LIN detector against the far engine's breaks - nine bits
  at neither length, thirteen at both, ten not at eleven.
- **Half duplex on a bus with a second chip.** HDSEL on the TX pad - an
  alternate-function open drain, TE clear - hears the four frames the far
  chip drives onto that wire from its receive pad, at 9600, 115200 and 1
  Mbaud; and the wired AND: this end's 0xF0 on its open drain, the far
  chip's 0x3C pulled into it as an emulated open drain on its pad's
  pull-up - the wire's only pull - and the far receiver reads 0x30.
- **IrDA between two WCH codecs** at 115200, eight bytes echoed exact
  through both decoders and both encoders; mute woken by an address mark
  (this node 5 among the far chip's three groups: the address frame 0x105
  and its three data) and by an idle line (muted after the first frame,
  awake for the next run).
- **The receive engine at 4.5 Mbaud**: a second of the far chip's
  gapless blast through each chip's receive ring - USART3's on DMA1's
  channel 3, UART4's on DMA2's channel 3 - 450000 bytes in sequence with
  no overrun, no frame error and no fault, the vector entered twice a
  512-byte lap (1757 edges for 450000 bytes) and never a byte.
- **The ring's counts and its edge, from the far chip** (key letter 9,
  each chip's receive ring on the link - USART3's on DMA1's channel 3,
  UART4's on DMA2's channel 3 - ten runs each way). Eight frames drawn by
  the far engine at 115200 with their stop cell low - every one a
  framing error carrying its byte - two bits apart: all eight stored in
  order and FE counted four, the every-other-one of the header's rule,
  nine vector entries; twelve bits apart, an idle line after each frame,
  the same four: the vector, waiting for a frame after each counted
  error with IDLEIE down, never sees those idle lines (the next frame's
  read clears the IDLE that stood, and the idle that frame armed is
  forgotten). A burst of one frame at 1 Mbaud told, its byte there,
  three of three; sixteen frames read whole on one USART interrupt, the
  edge 1642 to 1671 cycles after the last byte landed in the ring (1.1
  frames). `rx_pending()` read 37 with the far chip's 37 frames arrived,
  27 after ten were read and none after the rest, on the ring and on the
  interrupt receiver alike. A status read on the wait costs the count
  whole: the same ring reading STATR after the wait for a frame - for an
  interrupt transmitter's TXE, with TXEIE down - arms the clear the next
  frame's read performs, and after one counted error a run of errored
  frames counts no more (FE 1 of 8, both spacings, both parts, measured
  with that read in place); which is why the wait asks CTLR1 for TXEIE
  and reads STATR only when the interrupt transmitter is live.
- **UART4's engines on DMA2, on its second column** (the CH32V303VCT6):
  a message round the far chip's echo byte for byte; four kilobytes out of
  the transmit engine at 921600 counted clean and summed exact by the far
  chip, the run 998 thousandths of its wire time on this core's counter
  and +1601 ppm on the far one's (the divisor's +1602); four kilobytes in
  from the far blast, every byte in order and nothing counted. The
  transport's `remap` parameter on a column other than the default is
  what carries all of it (UART4's code 1).
- **The lot's registers on the wire** (the CH32V303VCT6): with CTLR4
  stored raw, UART4's zero bytes reach a far even receiver under MARK and a
  far odd one under SPACE with no parity error - the bit computed, as
  without the register; with M_EXT's five-bit code stored raw, zero words
  arrive as zero bytes, eight bits long; RX_BUSY read clear in 94953
  polls while the far frame arrived at 1200 baud.

### This part as the desk's instrument: usart_peer

One peer for the stratum, on the port and the timer the part links:
USART3 with TIM2 on its code 10 (CH3 PB10, CH4 PB11, DMA1's channels 1
and 7) on the CH32V203C8, UART4 on its second column with TIM3 on its code
00 (CH3 PB0, CH4 PB1, DMA1's channels 2 and 3) on the CH32V303RC and VC -
the timer's channels 3 and 4 on the port's own two pads. It runs the
crystal at 144 MHz (PCLK1 72, the timers 144: six core cycles a 24 MHz
protocol tick, so every span and edge converts exactly), answers
`Ident::xtal` = 1 and adopts nothing at `calibrate` (a crystal peer
reports the DUT's rate against its own), states its `send` and `blast`
rates from its divisor, and serves the protocol's ops but onewire (no
transmitter of the peer reaches the DUT's transmit wire but through the
peer's receive pad, which drive_rxd and collide drive instead), the
synchronous modes and a clock pad (no CK on the link), the five- and
six-bit words and clk2x (no OVER8 on this block).

What the silicon taught the peer's bring-up (a scratch probe, both
chips, the far board dark):

- **An enabled transmitter owns its pad.** With the TX pad in
  alternate-function push-pull and a timer's channel enabled on the same
  pad, a port with UE and TE set holds the pad at its own level - the
  timer's forced low does not reach it; with TE clear (or UE, or the port
  in reset) the timer's channel drives the pad; with neither driving, the
  CH32V303's pad reads low and the CH32V203's high. So the waveform
  engine's tenure parks the pad as a GPIO output high, forces the
  timer's channel high on a running counter, clears TE, and only then
  hands the pad back to the alternate function; the way back is the same
  steps reversed, TE set again first (its idle frame is high). USART3
  has no other column the LQFP48 bonds, so nothing is remapped.
- **No low pulse as TE rises**: the TX pad sampled every 8 cycles for
  228 us across a hand-over, TE set before or after the alternate
  function's nibble - no low sample in 4096, on both chips. A port is
  opened with the pad parked high across its reset, and needs no wait
  after TE.
- The timer's own facts - a forced level or a toggle waits for a running
  counter, a counter that never ran holds its output low, the compare
  request streams edges 14 cycles apart and misses one 12 apart, the one
  pulse returns the line high by itself - are [tim.md](tim.md)'s.

The offer, used or declined for the instrument:

| offer | answer |
|---|---|
| DATAR, one level, polled | the peer's loops: no interrupt, nothing masked; the receive turn 14 instructions with two APB loads (STATR, DATAR), the echo's 22 with four, an idle turn 12 with one; DATAR read before the first and the last frame's stamp, which is a call (a frame completing between the status read and the data read is lost with ORE cleared unseen, 18.10.1's pair) |
| the timer's toggle mode with CCxDE (15.4.7) | the waveform engine: an edge list as absolute compare values streamed by DMA into CHxCVR on each match - one beat an edge, no CPU between edges; the counter free over its sixteen bits, an edge's value modulo the counter, so a list is bounded by its table (640 edges) and not by the counter, its prescaler the smallest that keeps its longest interval under 0xF000 counts (one core cycle a count for every frame a UART sends); the floor 16 cycles between two edges |
| OPM with PWM mode 1 (15.3.10) | the glitch: one pulse exact to a count |
| a timer's update request writing BSHR | declined: a fixed sample rate is one table entry a sample, some hundred a bit for a 1 % pulse, where the toggle list costs one entry an edge |
| SBK | declined for `brk` and the auto-baud break: ten or thirteen bits only, and the transmitter sends its last byte again after it (the bench findings) |
| CTLR2.STOP's four codes | served as the register takes them; the halves leave as whole stops (above) |
| IREN, IRLP and the prescaler | served |
| M_EXT, OVER8, CK | refused: absent on this desk's die, not on this block, not on the link |

**The cost**, counted in the release listing and measured on the pair:
the echo at 4.5 Mbaud - a frame every 320 core cycles - takes 22
instructions and four APB accesses a frame; `send` and `blast` are
flattened (the protocol's pattern stream is out of line at -Os, a call a
frame), the stream's few instructions, the TXE poll and the store a
frame; the receive ring of the far DUT under the peer's blast took 450000
bytes in a second with nothing lost. The engine costs no CPU between
edges and one DMA beat an edge.

## Types and verbs

[brio/ch32vx03/usart.hpp](../../brio/ch32vx03/usart.hpp), two strata:

- The vocabulary: `UartBits` (seven, eight, nine data bits),
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
  `init(clock, baud)`, `isr()` (the edge), `write_byte()`/`write_bulk()`
  (a run: as many as fit, its first byte pushed and TXEIE armed before
  the rest is copied and armed again behind it; with an engine the run
  queued whole and the engine pumped once), `read_byte()`/`read_span()`/
  `consume()` (the receive run in place; with a receive engine
  `consume()` answers whether the run was intact), `rx_pending()`/
  `tx_idle()`, the counters (`rx_overruns()` - a byte a full ring refused,
  or with a receive engine a lap or a held run the ring skipped -,
  `frame_errors()`, `parity_errors()`, `noise_errors()`,
  `hw_overruns()`, `clear_errors()`), `baud()`/`actual_baud()`/
  `set_baud()`/`rebase()`/`divisor_for()`/`can_baud()`/`min_hz_for()`,
  `release()`, `rx_skips()` (the gaps in the stream the receive ring
  hands out, never cleared - util/line_reader.hpp's epoch: without an
  engine the SkipRing's skips, one at the consumer's look after `isr()`
  dropped a flagged byte or a full ring refused one; with one the view's
  skips plus an overrun and a restart after a transfer error), and the
  engine
  verbs `dma_isr()` (whose true is the receive edge, as `isr()`'s),
  `harvest()` (the same edge asked from the consumer's side - true when
  the ring holds bytes and the consumer has found it empty since the last
  true from a vector or from here - and a stopped ring started again; it
  reads neither STATR nor DATAR), `dma_faults()` (blocks a transfer
  error threw away, and receive channels given up on a frame they never
  took). `tx_idle()` is the
  wire's: the ring empty, no block in flight, and TC, cleared as every
  transmit block starts. `write_bulk()` copies a run of 16 bytes or more
  whose source and ring slot share their alignment with the runtime's
  memcpy, any other with its byte loop. The frame is its own parameter
  here, ahead of the engine slots, and `UartOptions` - the trailing one -
  carries what is left: `half_duplex` (the TX pad as an alternate-function
  open drain, the RX pad untouched), `rts`/`cts` (the column's pads,
  refused on an instance that is not full and on a package that does not
  bond them) and `rx_priority` (the receive engine's level, very_high by
  default, refused without a receive engine). A remap code of 0 writes no
  AFIO register at all.
- `tx_room()` is the bytes `write_bulk()` would take now, the transmit
  ring's free count (util/stream.hpp's `RoomSink`,
  [design/serial.md](../design/serial.md)): it grows a byte at a time as
  the TXE entry hands DATAR the ring's bytes, and a block at a time under
  a transmit engine, whose block holds its run in the ring until its
  completion. `tx_drained()` is the transmit edge (`EdgeSink`), true once
  each time the ring ran dry: raised by `isr()` on the TXE entry that
  finds the ring empty and disarms, the last byte then in the shift
  register, or by `dma_isr()` on the transmit completion that leaves the
  ring empty, and read by the program's glue right after that body, in
  the same vector (the instance's, or the transmit channel's).

## How to use it

The console on USART1's default pads, which both boards wire to the
probe's serial:

```cpp
using Serial = brio::Uart<1, P>;                 // USART1, PA9/PA10, 8N1
constexpr Serial serial;
Serial::init(clock, 115200);
BRIO_CH32_VECTOR(usart1_handler) { (void)Serial::isr(); }
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
BRIO_CH32_VECTOR(uart6_handler) { (void)Aux::isr(); }
```

The transport with both DMA engines, on the slots this instance's
requests are wired to - DMA1's for USART2, DMA2's for UART4 on the
CH32V303:

```cpp
using Loop = brio::Uart<2, P, 256, 256, brio::UartFormat{},
                        brio::DmaTxEngine<1, 7>, brio::DmaRxEngine<1, 6>>;
BRIO_CH32_VECTOR(usart2_handler) {
    if (Loop::isr()) { brio::post<LoopLines>(brio::RxActivity{}); }       // idle line, first frame
}
BRIO_CH32_VECTOR(dma1_channel6_handler) {
    if (Loop::dma_isr()) { brio::post<LoopLines>(brio::RxActivity{}); }   // the lap's marks
}
BRIO_CH32_VECTOR(dma1_channel7_handler) { (void)Loop::dma_isr(); }

using Far = brio::Uart<4, P, 256, 256, brio::UartFormat{},
                       brio::DmaTxEngine<2, 5>, brio::DmaRxEngine<2, 3>>;
BRIO_CH32_VECTOR(uart4_handler) { (void)Far::isr(); }
BRIO_CH32_VECTOR(dma2_channel5_handler) { (void)Far::dma_isr(); }
BRIO_CH32_VECTOR(dma2_channel3_handler) { (void)Far::dma_isr(); }
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

The reference suite is `test_vx03_serial`, at 144 MHz on each board's
crystal - PCLK2 144, PCLK1 72 - with USART2 as the instrument on its
default pads and the console untouched on USART1: 64 verdicts in `z` on
the CH32V203C8T6 with the crossed pair strapped and 44 on the
CH32V303VCT6 (its letter p that series' own), the far board dark; and the
two-board key `y` against the far chip's usart_peer (the pair's section
above): 67 verdicts on the CH32V203C8T6, 80 on the CH32V303VCT6 with its
letters n and o. Almost every
letter needs no wire, and three facts of this silicon are why: the
receive pad is a bit-banged transmitter (an input follows its own pull,
which the output data register moves in a few cycles), a pad driven by a
peripheral still reaches a TIMER CAPTURE input on the same pad (USART2's
TX is TIM2's channel 3, the fourth port's remapped CK is TIM3's channel
1), and the fourth serial port is a USART whose clock pad can be counted.
Every fact above with a number is this suite's measurement. The letters
that want a strap detect its absence and say so: the loopback PA2 to PA3, and the
crossed pair of USART2's pads with UART4's default ones, PA2-PB1 with
PB0-PA3 on the CH32V203C8T6's board - measured below; the CH32V303
evaluation board's pair is retired, its UART4 default pads PC10/PC11 being
the desk pair's SPI link, and that part's letters m, q and s with it.

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
- **THE RATE ON THE WIRE, timed on the core's counter.** A loop and the
  crossed pair divide one bus, PB1, by divisors from one arithmetic at
  both ends, and TIM2 - the frame ruler above - counts a clock of the
  same bus, so none of them can tell a wrong bus rate; the STK counts
  HCLK. Every letter that loops or crosses a wire, letter b and letter p
  send a run of frames back to back - through the bare port, a store
  whenever TXE stands, or through a transport's `write_bulk()` - timed
  from the first store to the last stop bit's end (TC, `tx_idle()`), in
  thousandths of the run's time at the rate ASKED, judged -1 % to +3 %.
  The same on both parts unless one is named:

  | letter | port, path | rate, frames | reading |
  |---|---|---|---|
  | b | USART2 bare, 8N1 | 9600 x 16, 115200 x 256, 921600 x 1024, 3 Mbaud x 1024 | 1006, 1000, 998, 1000 |
  | m (CH32V203C8T6) | USART2 and the fourth port bare | 8N1 115200 x 256; 8E2 19200 x 32 | 1000 and 1000; 1000 and 1001 |
  | q (CH32V203C8T6) | the fourth port bare; USART2's transmit engine | 115200 x 256 | 1000; 1000 |
  | r | USART2's interrupt transmitter; its transmit engine | 115200 x 256, 1 Mbaud x 1024 | 1000, 1000; 1000, 1000 |
  | s (CH32V203C8T6) | the fourth port bare; USART2's transmit engine | 1 Mbaud x 1024 | 1000; 1000 |
  | n (CH32V303VCT6) | UART4's engine on DMA2, into the far chip | 921600 x 4096 | 998 |
  | o (CH32V303VCT6) | USART2 bare, UART4 bare into the far chip, 8E1 | 9600 x 16 | 1001 and 1004 to 1005 |
  | p (CH32V303VCT6) | UART5 to UART8 bare | 9600 x 16 | 1001 each |

  The 998 and 999 at 921600 are the divisor's own rounding - 78 of a
  72 MHz bus gives 923077 baud, 0.9984 of the wire. The 1001 to 1006 at
  9600 and 1200 are a run of sixteen frames, where a thousandth is a
  sixth of a bit: they are what the first start bit would cost if it
  waits for the generator's next bit tick, up to a bit - not measured
  apart. A run is started on a port whose previous frame has ended by
  TIME: TC stood from a frame before, a write that follows no STATR read
  leaving it set, and a run started on it read a whole frame long (1056
  at 9600). The suite runs on the board's crystal, so the counter's
  cycles are the crystal's; what a counter on the same chip cannot tell -
  its own clock's error, which moves the wire and the ruler together - is
  the far chip's to time, and the pair's section above has it: every rate
  within a few ppm of PCLK1 over BRR on the other board's crystal.
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
  and no error flag on either side - the CH32V203C8T6's board.
- **Errors under the receive engine** (letter q, the CH32V203C8T6's
  crossed pair, the fourth port the sender): 120 data bytes with 14
  breaks between them in a continuous stream - every data byte delivered
  intact and in order, the 14 breaks stored as the 0x00 frames they are,
  FE counted 14: no byte taken by a clear. Two breaks back to back count
  1, three count 2. The interrupt receiver drops each break's frame,
  counts it and skips it: read between the breaks 40 of 40, FE 10 for
  10, ten skips; read after them, 8 bytes around a break, none handed
  out and one skip. The breaks are banged on the
  sender's pad: measured on the same board, the transmitter SENDS ITS
  LAST BYTE AGAIN after an SBK break, so SBK put a repeated byte into
  every stream it broke - a fact of the sender worth knowing, not of the
  receiver.
- **`tx_idle()` is the wire's** (letter r, TIM2's channel 3 latching
  PA2's last rising edge): its first true lands 1263 to 1295 timer clocks
  after the last stop bit starts at 115200 (a bit is 1250) and 151 to
  186 at 1 Mbaud (a bit is 144), on the interrupt transmitter and the
  transmit engine alike - TC at the stop bit's end, a poll's few clocks
  late. Before this round the engined transport could answer from a TC
  the frame before its block had left.
- **The burst edge from the vector** (letter s, nothing polled): every
  burst of one frame told; a burst of 16 at 1 Mbaud told about 1711
  cycles after its last stop bit (1.1 frames) with two USART interrupts
  and none a byte; four laps of the 256-byte ring with no silence read
  whole on the lap's marks.
- **The cost** (`bench_vx03` letter u on the CH32V203C8T6, USART2 on the
  crossed pair, the fourth port's engine the stimulus; the CH32V303's
  letter u declines by name, its pair retired): the receive engine takes two USART
  interrupts a burst and one channel interrupt a half lap, each entry
  about 165 to 190 cycles between the bench's stamps - the idle-line one
  reading STATR once, CNTR twice and writing CTLR1 and CTLR3, the
  first-frame one CNTR alone and the same two writes; the edge lands
  13497 cycles after the last stop bit at 115200 (1.0 frame), 1705 at 1
  Mbaud (1.1) and 497 at 4.5 Mbaud (1.5), where the BEFORE - the owner
  asking `harvest()` once a tick - was 50424, 120624 and 124450. A burst
  of 16 at 1 Mbaud is read in 1.11 of its wire time (6.37 before). The
  interrupt receiver is one entry a byte, about 150 cycles each. Transmit
  is wire-bound through either path at 115200, 1 Mbaud and 4.5 Mbaud (x
  1.00); the engine's interrupts are one a block, but the core does not
  sleep while a channel runs on this family ([dma.md](dma.md)), so a
  print through it costs the core its sleep, not its cycles.
- **Against the vendor** (the EVT's `USART/USART_Idle_Recv`, openwch
  ch32v20x baef005, moved to USART2 on the same pair and bursts, a
  scratch program): its idle-line handler is one interrupt a short burst,
  324 cycles - against brio's two of about 165 - and three for 256 bytes,
  2144 cycles, because it copies every block out of a pair of one-shot
  buffers into a ring of its own inside the handler (brio's 256 bytes:
  three interrupts, 434 cycles, the consumer reading the channel's own
  ring in place); its edge lands at the same frame (13613, 1871 and 613
  cycles at the three rates). Its clear of IDLE is `USART_GetITStatus`
  then `USART_ReceiveData` - a read of DATAR while the channel it has
  just re-armed owns it.
- **A receive channel that never takes the frame** (`test_vx03_dma`
  letter p, both parts): USART2's ring armed over channel 6 with TIM3's
  compare request held there by a gate closed without a reset, the
  channel's enable moving one item (count 127 of 128) and then nothing;
  four frames banged in: the vector entered 256 times - the overrun's
  entry and the 255 of the wait - gave the channel up with one DMA fault
  counted, and the thread ran on; with TIM3's reset pulsed the consumer's
  next look started the ring again and ten frames arrived whole.
- **An overrun at a message's tail** (`test_vx03_dma` letter r, both
  parts: on the CH32V203C8T6 USART2's ring fed by the fourth port at 1
  Mbaud, on the CH32V303VCT6 UART4's ring on DMA2 fed by the far chip's
  usart_peer, the copy on DMA2's channel 1): a 2 KB copy at
  the ring's own level on a lower channel starving it under a message's
  last frames, the vector held off until the channel had taken the frame
  DATAR held - ORE up, RXNE down, no frame after it. The vector entered
  THREE times (the overrun's entry, the frame not yet taken, the clear),
  no channel was given up, the overrun and the gap were each counted
  once, the 28 bytes that landed before the copy arrived in order with
  the frame DATAR held (29 bytes on the CH32V303, whose sender is the
  far chip and ran on through the copy; 31 on the CH32V203,
  whose sender the copy starved too, so that only one frame overran),
  and the next message arrived whole with nothing else counted. THE CONTROL, the same
  image with the clear disabled: 256 entries, the live channel given up
  as a DMA fault, the bytes received before the overrun thrown away by
  the restart, the overrun counted twice - and the NEXT message lost as
  well, the restart's own entry starting a second storm that gave the
  channel up under it.
- **The clear's window, staged wide** (the same letter): a frame landed
  while a copy starved the ring, and the CPU read STATR and DATAR under
  it, as the clear would read a frame landing between its two loads. The
  channel moved nothing while starved and ONE item when the copy let it
  go, and the ring then held the frame: the request a frame raises
  outlives a CPU read of its data, so the byte is not lost, and only its
  error flags go with the clear.
- **The ring against a copy** (`test_vx03_dma` letter q, both parts): 16
  KB at 1 Mbaud and 64 KB at 4.5 Mbaud poured into a ring of 512 while a 2
  KB copy ran back to back on a lower channel at the copy engine's default
  level - on the CH32V203C8T6 the fourth port into USART2's ring on DMA1's
  channel 6 beside a copy on channel 3, on the CH32V303VCT6 the far chip's
  blast into UART4's ring on DMA2's channel 3 beside a copy on DMA2's
  channel 1: every byte in order, no overrun of either kind, no channel
  given up, the vector entered once; the copy at 0.96 to 1.00 of its rate
  alone at 1 Mbaud and 0.92 to 0.98 at 4.5 Mbaud. The ring told
  `rx_priority = low` under the same copies: some nine hundred overruns in
  8 KB at 1 Mbaud, about a thousand bytes of 8192 delivered, and the
  vector entered 145 to 155 thousand times in 85 ms, a starved channel
  re-entering it back to back.
- **The receive ring** is `test_vx03_dma`'s letter g, with no wire: the
  channel circular over the storage from `init()`, a burst across the
  storage's end delivered whole with nobody reading, a lap the consumer
  missed counted once and skipped, a held run refused at its release
  ([dma.md](dma.md)).

## Not covered yet

Driver gaps, each with its reason:

- **Exact error counts under the receive engine**: bounded, not exact
  (above) - an exact count would need the CPU's read of DATAR, which the
  engine's rule forbids; declined.
- **The receiver's bit under partial noise**: the census above finds
  the three instants and their spread, but which way the bit goes when a
  pulse covers one or two of them is not settled by one pulse a frame of
  an unknown phase - a train with TWO pulses in a frame, one on each of
  two instants of a phase the frame itself fixes, is what would, and
  usart_link.hpp's stage draws one; no verdict reads the word there.
- **The half duplex's own echo on a held wire**: HDSEL hears a second
  chip's frames on the pair's wire (above), and the instance's own frames
  are what the single-board letter f prints and does not judge - the wire
  is released between frames with no resistor on either board; one to
  the supply on the TX pad is what would make that echo measurable.
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

Implemented but not bench-verified, each with what would measure it:

- **`tx_room()` and `tx_drained()` on this silicon**, on both series:
  the same ring and the same dry branch are judged on the host over the
  simulated PL011 (test/test_pl011) and measured on the STM32F4's
  silicon ([the STM32F4 USART page](../stm32f4/usart.md), letter h); a
  letter that writes a run with interrupts masked, reads the room under
  the mask and the edge after the wire time would measure them here.
- **Letter l's rate on the wire**: the loopback's 921600 run through the
  transmit engine is timed against the wire like every other loop, and
  neither board carries the PA2-PA3 strap while the crossed pair is
  wired; the strap in place of the pair is what would run it.
- **The lot's four features on a die that has them**: the MARK parity
  judged by the far chip's parity check, the short words reaching its
  8N1 receiver with their top bits the stop's, and RX_BUSY under its
  frame are letter o's branches for a die that has them, and the
  CH32V303VCT6 answered no to each; what would measure them is a
  CH32V303 of a lot whose penultimate sixth digit is not zero.
- **The debug-port columns once the port is given up**: the refusal is
  measured with the probe attached; a program that gives the port up
  and then brings UART8 up on PA14/PA15 is what would measure the other
  side of it - with no probe attached.
- **Errors separated by a clean frame under the ring, from the far
  chip**: key letter 9 draws trains in which every frame carries the
  error (the far engine's pulse sits in every frame); a train mixing
  clean and errored frames is letter q's, on the CH32V203C8T6's crossed
  pair - the far chip's stage drawing a pulse in some frames only is what
  would measure it on the CH32V303VCT6.
