# USART (CH32X035)

This document is for a programmer who needs a serial port: the console
a program prints to, a line-oriented link, a byte transport to another
chip, or one of the modes the peripheral offers (LIN, IrDA, the
smartcard, a synchronous clock, half duplex on one wire). It covers the
four USARTs of the CH32X035 series in the two layers every brio serial
driver has ([design/serial.md](../design/serial.md)): `Usart<n>`, the
resource, which instance, its clock gate, its vector, its DMA channels
and chapter 14's registers as functions, one per register fact; and
`Uart<n, P, ...>`, the task (a task, in brio, is a driver layer with
state of its own, served from its peripheral's interrupt, that an
active object or `print` calls; an active object is a component of a
brio program with its own event queue, served one event at a time by
the kernel, [docs/learn/introduction.md](../learn/introduction.md)): the
interrupt-driven byte transport every console runs on, with a transmit
ring and a receive ring, a DMA engine in either slot, and the same
public surface and template parameters as the sibling strata's tasks,
which is what lets `util/serial_port.hpp` and `print()` run here
unchanged.

The smallest use, the console on the evaluation board's USART2 (its
default column, PA2 and PA3, wired to the probe's own serial):

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

Line by line: `P` is the platform ([platform.md](platform.md)), which
the rings ask whether an index can be shared with a handler bare;
`Serial` names the transport type, `2` the instance and every other
template argument at its default (the ring sizes, 8N1, no DMA engines,
the reset column, the default options, the flash placement); `serial`
is an empty tag object of that type, the argument `print` takes to pick
the port, since the type's functions are all static and it has no
object of its own; the `extern "C"` function is the interrupt handler,
as in C, under the crt's name for USART2's vector and with
`BRIO_CH32_INTERRUPT` in front (the one spelling of a handler attribute
on this target, [platform.md](platform.md)); `isr()` is the port's
whole interrupt body and returns true when a received byte made the
receive ring non-empty, the edge the program posts as `RxActivity` to
the active object that assembles lines (`SerialLines`, a `SerialPort`
of `util/serial_port.hpp`; a program with no such object discards the
answer); `init()` takes the clock task's tag (`constexpr SysClock
clock;`, [clock.md](clock.md)) and the baud rate; `brio::crlf` is the
line ending.

What decides correctness even there: `init()` after the clock task's,
since the baud divisor is computed from the rate the clock type claims
and not read from the hardware, so a port initialized on a chip still
at its 8 MHz reset rate with a 48 MHz clock type speaks at a sixth of
the rate asked, and nothing refuses it; the instance must be one the
part offers (the 20-pin CH32X035 parts have no USART1 column with both
pads, so `Uart<1, P>` does not compile there, "What the silicon does"
below); and the instance's vector must be bound to `isr()` by the
program, as above - a port whose vector is left unbound raises its first
transmit interrupt into the crt's default handler, a spin loop, and the
program stops there. Three more things to know before the first
`print`: `print` waits until the transmit ring has taken every byte, so
a long message printed from a handler stalls every other active object
for that long ([design/serial.md](../design/serial.md)); the transmit
ring has one producer, so `write_byte` and `write_bulk` are called from
active objects and from `main`, never from an interrupt handler, and
both copy, the caller's buffer being free when the call returns; and a
column on the debug port's pads (USART3's only column on the 20-pin
parts) is refused by `init()`, which answers false while the probe owns
them.

The page is long because the chapter is. In order: the documents of
record; what the silicon does; the types and verbs; how to use it, with
the DMA engines and the placement in SRAM; the bench findings, most of
the page; this part as the desk's instrument for the sibling board's
suite; what is not covered.

## Documents of record

The CH32X035 reference manual V1.8 (14.1 for what
the block has, 14.3 for the divisor, 14.4 to 14.7 for the synchronous,
half-duplex, smartcard and IrDA modes and what each excludes, 14.8 and
14.9 for the DMA and the interrupts, 14.10 for the registers; 8.3.2.1
for the remap columns, 9.2.3 for the DMA requests, 3.4.6 and 3.4.7 for
the gates, table 7-1 for the vectors) and the CH32X035/X033 datasheet
V1.7 (the model table's count of serial ports per part, table 2-1 for
the pads each package bonds). The header is
[brio/ch32x035/usart.hpp](../../brio/ch32x035/usart.hpp), with the
columns in [afio.hpp](../../brio/ch32x035/afio.hpp), the request
channels in [dma_engine.hpp](../../brio/ch32x035/dma_engine.hpp) and the
engines in [dma.hpp](../../brio/ch32x035/dma.hpp) ([dma.md](dma.md)); the
reference suite is `test_x035_usart` (a suite's letter, below, is one
command key of its console and the group of verdicts it runs; `z` is
the key that runs the single-board set and `y` the two-board set), and
`test_x035_dma` runs its own console on the engines.

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
  output mode plus pull" - an alternate push-pull output with an
  external pull, the only alternate output this series has - with LIN,
  the smartcard, IrDA and the clock off. 14.5's "the TX and RX pins
  connected internally on the chip" is the receive path from the TX
  pad: it hears another node driving that pad while the pad is an input,
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
    CPOL and CPHA, and LBCL - which this manual describes as "1: the clock
  pulse for the last bit of data is not output" (14.10.5), where WCH's
  own library names the same bit `USART_LastBit_Enable`. CPOL, CPHA and
  LBCL cannot change once the transmitter is enabled; the smartcard,
  half duplex and IrDA off.
- **Flow control** (14.10.6): RTSE and CTSE, CTS with its flag and its
  interrupt.
- **DMA** (14.8, 9.2.3): DMAT and DMAR, each direction on a channel of
  its own - USART1 on channels 4 (TX) and 5 (RX), USART2 on 7 and 6,
  USART3 on 2 and 3, USART4 on 1 and 8. Under a receive channel the CPU
  never reads DATAR, and the error flags' clear is the silicon's (measured,
  below): a STATR read arms the clear 14.10.1 prescribes, and the next
  DATAR read - the channel's - performs it; a flag with no status read
  since the last DATAR read stands. The transport's receive vector is
  built on that ([../../brio/ch32x035/usart.hpp](../../brio/ch32x035/usart.hpp)'s
  header).

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

`Uart<n, P, rx_size, tx_size, format, TxEngine, RxEngine, remap, opts,
Placement>`, the transport - the CH32V203's parameter list, then
`UartOptions` (`half_duplex`, `rts`, `cts`, `rx_priority`), then where
the port's hot paths run from, util/placement.hpp's `InFlash` (the
default) or `InRam` - the per-byte `isr()` and the transmit engine's
block restart; `InRam` is refused on a platform that does not place
`.fast_text`. It refuses at
compile time an instance the part does not offer, a nine-bit or an
invalid format (the rings carry bytes; nine bits are the resource's
`read_word`), a column that is not a transport's on this part, an RTS or
CTS pad the column's package does not give, an engine on any channel but
the instance's own (the request table's, [dma.md](dma.md)), two engines
on one channel, a receive ring under an engine that is not a power of
two, and `rx_priority` on a port with no receive engine.
`init(clock, baud)` answers false when the divisor comes out below 16,
and when the column puts TX or RX on the debug port's pads while the
probe's port is alive; otherwise it opens the gate, writes the column
if it is not the reset one, hands TX to the peripheral before the enable
and leaves RX a floating input, configures the frame and the divisor,
writes CTLR3 only where an option or an engine has something for it -
DMAT, DMAR with EIE -, binds the engines (the receive ring at
`rx_priority`, very_high by default, the transmit at high), and arms
the vector: RXNE without a receive engine, the idle line and the errors
with one. Then: `isr()` (the vector's whole body, true on the edge from
an empty receive ring), `dma_isr()` (the engines' channels' body: a
transmit block's completion releases its bytes and starts the next run, a
receive lap is counted, the ring's half and full marks are the edge, a
transfer error stops the channel for the consumer's next look to
restart), `harvest()` (the same edge asked from the consumer's side),
`dma_faults()`, `write_byte` and `write_bulk` (a run: as many as fit -
without a transmit engine its first byte pushed and TXEIE armed before
the rest is copied, armed again behind it; with one the run copied whole
and the engine nudged, the block claimed under the mask and launched
outside it), `read_byte` and `read_span`/`consume` (the receive run in
place - under a receive engine the channel's own ring, `consume()`
answering whether the run was intact), `tx_idle()` (no block in flight,
the ring empty, TC up - each block clears TC as it starts), `rx_pending()`,
`rx_skips()` (the gaps in the stream the receive ring hands out, never
cleared - util/serial_port.hpp's epoch: without an engine the SkipRing's
skips, with one the view's skips and the overruns and restarts the vector
and the look count); the counters `rx_overruns` (a full ring, or a lap of
the channel's ring written over unread bytes), `hw_overruns`,
`frame_errors`, `noise_errors`, `parity_errors` (saturating) and
`clear_errors()`; `baud()`, `actual_baud(hclk)`, `divisor_for`,
`min_hz_for`, `can_baud`; `rebase(hz)` - the port as a `ClockUser`,
draining what it holds at the old rate before writing the new divisor -
`set_baud(hz, baud)` for the link's own rate, and `release()`, which
stops the engines, clears UE and pulses the block's reset line before it
closes the gate: a raised request outlives every other release on this
die ([dma.md](dma.md), "A released requester").

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

USART4 on its two DMA engines - it transmits on channel 1 and receives on
8 (entry 44 of the vector table) - with a receive ring of 512, a power of
two, written by the channel lap after lap:

```cpp
using Link = brio::Uart<4, P, 512, 256, brio::UartFormat{}, brio::DmaTxEngine<1>, brio::DmaRxEngine<8>>;

extern "C" BRIO_CH32_INTERRUPT void usart4_handler() {
    if (Link::isr()) { brio::post<LinkLines>(brio::RxActivity{}); }
}
extern "C" BRIO_CH32_INTERRUPT void dma1_channel8_handler() {
    if (Link::dma_isr()) { brio::post<LinkLines>(brio::RxActivity{}); }
}
extern "C" BRIO_CH32_INTERRUPT void dma1_channel1_handler() { (void)Link::dma_isr(); }
```

The block restart in SRAM, gapless to 3 Mbaud (Bench findings, letter
`s`), is the port's placement:

```cpp
using Link = brio::Uart<4, P, 512, 256, brio::UartFormat{}, brio::DmaTxEngine<1>, brio::DmaRxEngine<8>, 0,
                        brio::UartOptions{}, brio::InRam>;
```

A program that ranks a copy of its own above the ring says so - and loses
the ring's frames to it, which the rule exists to prevent:

```cpp
using Low = brio::Uart<4, P, 512, 256, brio::UartFormat{}, brio::DmaTxEngine<1>, brio::DmaRxEngine<8>, 0,
                       brio::UartOptions{.rx_priority = brio::DmaPriority::low}>;
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
leaving their link pads floating inputs; `y` (98 verdicts) is the
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
- **The rate asked, on the transmitter's own clock** (letter `d`, a run
  polled one frame ahead of the shifter, from the first store to TC on
  the ticker's cycle count, against its wire time at HCLK over the baud
  asked): the console's 128 frames at 115200 in 1001 thousandths (BRR 417
  for 416.7 is 0.8 slow), and USART4 on PB0 1024 frames in 1000 at
  115200 and at 1 Mbaud and 1001 at 3 Mbaud, a divisor of exactly 16 -
  with the receive pad pulled up: it is a wire to the far board's
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
- **The HSI against a crystal** (letter `j`): the far chip timed a
  gapless run of 4000 frames at 1 Mbaud - a divisor of 48, so the HSI's
  error alone - first frame to last on its crystal's clock: -2480 to -2706 ppm at the
  factory calibration (HSICAL 0x53, the trim at 16) over three sessions'
  runs, the oscillator wandering by tens to a few hundred ppm between
  runs a fraction of a second apart (every later letter's reference is
  measured back to back with what it judges). The trim stepped from 7 to
  28 and each step timed the same way: -25610 ppm at 7, -292 at 17 (the
  trim nearest 48 MHz on this die), +25302 at 28; every step up, between
  71 and 183 kHz, the mean 116 to 118 kHz against the manual's "about
  110KHz" (3.4.1), and back at 16 within one step of the first reading.
  The same letter's rendezvous holds the peer's stand-off: a standby of
  300 ms acked while the line is still the peer's, a ping inside it
  unanswered, and the first ping answered 304 ms after the ack.
- **The rates on the wire, timed by the foreign receiver** (letter `k`):
  at 115200, 921600, 1, 1.5, 2, 2.4 and 3 Mbaud sixteen bytes echoed
  equal and counted clean by the far chip, and 2000 gapless frames timed
  within 500 ppm of HSI / BRR - the 1 Mbaud reference taken just before
  each - where a wrong divisor anywhere on that ladder is 2400 ppm off or
  more: 115200 at -3778 ppm against -3746 (the divisor's -0.08 per cent
  on the HSI's), 921600 at -1436 against -1276 (the divisor's +0.16), 3
  Mbaud at -2830 against -2901.
- **The transport across the wire, on interrupts** (letter `k`): eight
  bytes through `Uart` echoed by the far chip come back equal at 115200
  and 1 Mbaud; 64 bytes it sends at 3 Mbaud are all counted by the far
  chip; a gapless run of 256 INTO it, taken by runs
  (`read_span()`/`consume()`), arrives whole and in order at 1, 1.5, 2,
  2.4 and 3 Mbaud, and the eight-byte echo at 3 Mbaud comes back whole -
  with the vector in SRAM, which the suite's program places
  (`.fast_text`, [platform.md](platform.md), "A handler in SRAM"), the
  port in its default placement, its body inline in that vector. The
  interrupt transport's ceiling is the placement's: with the same vector
  in the flash, the run arrived whole to 2 Mbaud, at 2.4 Mbaud it broke
  within its first 16 to 128 bytes (the receive ring full behind the
  main loop, or an overrun) and at 3 Mbaud within its first 21, and the
  echo at 3 Mbaud came back two to eight bytes of eight from run to run;
  with the port's body placed (`InRam`, 488 bytes for the suite's
  engined port) and the vector in the flash, whole to 2.4 Mbaud, and at
  3 Mbaud 32 bytes before an overrun. The transport takes one vector
  entry a frame - the hardware prologue's round trip and a receive body
  of about 30 instructions, no call when inline - against a frame of 160
  cycles at 3 Mbaud and 200 at 2.4: from the flash the entries leave the
  main loop too little of the frame, from SRAM they leave it enough, and
  the vector's own path in the flash costs the placed body the last
  step. The console's handler, the same body, measures 145
  cycles from the flash and 66 from SRAM (stamped in test_x035_spi), its
  vector 404 bytes there.
- **The same sweep on the engines** (letter `k`, USART4 transmitting on
  channel 1 and receiving on 8, the ring 512 bytes): 8192 gapless frames
  from the far chip at 1, 1.5, 2, 2.4 and 3 Mbaud, every byte judged in
  sequence - each run whole and in order, no overrun, no lap lost, no skip,
  no channel given up, and two USART interrupts a burst and the ring's two
  marks a lap (32 for 8192) where the interrupt transport took one a
  frame. 3 Mbaud, a divisor of 16 on both chips at 48 MHz, is the wire's
  ceiling here: the engined receive runs at the wire's, a factor of one
  and a half over the interrupt transport's 2 Mbaud from the flash. A full-duplex echo
  of 256 bytes at 3 Mbaud on both engines came back whole, no ORE. The
  engined receive's vectors run a few times a lap, so their placement
  does not bear on this.
- **The error flags' clear under the receive channel** (letter `q`): the
  far chip's 8N2 frames 0x03 and 0x04 into a receiver set for 8E1 - its
  first stop bit read as the parity bit, so 0x03 is a parity error and
  0x04 clean - taken by channel 8 from DATAR, both bytes landing: PE stands
  in STATR after the first frame although the channel read DATAR; with
  STATR read between the two, PE is down after the clean second frame -
  the status read armed the clear and the channel's read performed it;
  with no status read between, PE is still up after the second. The
  CH32V006's and the CH32V203's answer, which the transport's receive
  vector is built on. And a run of errored frames under a ring with no
  transmit engine (ten runs): eight frames of 0x35 from the far engine at
  115200, each with its last data cell stretched a whole bit over the
  stop bit's sample - a framing error carrying its byte - four bit times
  apart, into channel 8's ring beside the interrupt transmitter: all
  eight stored, FE counted four, the every-other-one of the header's
  rule, eight USART entries. The same ring reading STATR after the wait
  for a frame - for the interrupt transmitter's TXE, with TXEIE down -
  counted none of the eight (three of three runs, measured with that read
  in place), which is why the wait asks CTLR1 for TXEIE and reads STATR
  only when the interrupt transmitter is live.
- **The priority rule on this controller** (letter `r`): 8192 frames at 3
  Mbaud into the ring at very_high on channel 8 while 2 KB copies ran back
  to back on channel 5 - a LOWER number - at the copy engine's low: every
  byte in order, nothing lost, 395 to 409 copies beside it. The control,
  the ring told `rx_priority = low` - a tie, the copy's lower number
  winning it, a block of 512 words some 3 000 cycles against DATAR's one
  frame of 160: the run broke at its second byte, an overrun counted and
  the stream skipped.
- **The transmit engine's blast, timed by the far chip** (letter `s`):
  4096 frames through the ring of 256 - 48 blocks, each started from the
  last one's completion - against the polled blast taken back to back,
  in both placements of the port. The DMA vector's body - the flags'
  read, the channels' service and the restart (the ring's consume, the
  claim, the next run, TC cleared and the five stores) - is flattened, no
  call left on it, and placed whole with the port: under `InRam` the
  suite's vector in the flash keeps its entry, the owner test, the stamps
  and one call. Under `InRam` gapless at every rate to 3 Mbaud, twelve
  runs: the span within 500 ppm of the polled blast's at 1, 1.5, 2, 2.4
  and 3 Mbaud - the instrument's own spread, two blasts back to back on a
  wandering HSI, was -454 to +314 ppm over the 252 legs of 36 runs that
  idle nothing, and one cycle a block at 3 Mbaud is 73 ppm - and -2 to 3
  cycles a block at 3 Mbaud, USART4's own vector never entered during a
  blast; 129 to 132 cycles of body between its first and last statement
  (154 at most), 326 bytes of SRAM a port
  (`.fast_text.brio`), a transfer error's path a call out of it. The
  margin, measured by delaying the suite's vector before the body: 16
  cycles more left the 3 Mbaud blast within the instrument's spread, 24
  idled the line 5 to 8 cycles a block and 32 14 to 17 - the restart
  lands some 15 to 20 cycles inside its window at 3 Mbaud - and 32 more
  idled 0 to 7 at 2.4. From the flash (`InFlash`, the default) the same
  body inline in the suite's vector takes 285 to 289 cycles (308 to 324
  at most), the vector on a word as `BRIO_CH32_INTERRUPT` aligns every
  vector - half a word off, the same body took 276 to 283, the vector's
  place in the flash moving its cost ([platform.md](platform.md), "A
  handler in SRAM"): the alignment buys a figure that stays, not always a
  lower one. Gapless to 1.5 Mbaud, and above it each block
  boundary idles the line for what the restart outlasts that window - 12
  to 20 cycles a block at 2 Mbaud, 47 to 57 at 2.4 and 98 to 107 at 3,
  the span 0.06 to 0.77 per cent over the polled blast's - and for no
  more (the suite's verdict). Why the whole
  body: the restart alone placed, the flags' read and the service left
  inline in the suite's flash vector, measured 172 to 174 cycles and
  idled the line 3 to 10 cycles a block at 3 Mbaud, outside the 500 ppm
  in 2 runs of 6; the suite's vector itself placed in SRAM measured 116
  to 117 cycles and -1 to 3.
- **The frame matrix across the wire** (letter `l`): 8N1, 8E1, 8O1, 7E1,
  7O1, 9N1, 8N2, 8N0.5 and 8N1.5 at 115200, eight frames each echoed
  equal and counted clean; half a stop bit and one and a half timed as
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
  DATAR keeping the first frame (0xB0) and the seven after it lost. A
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
  chip's codec, and in low-power mode both codecs at PSC 26 (48 MHz into
  1.846 MHz, the SIR low-power clock's usual 1.8432 MHz within 0.2 per
  cent, a pulse of three of its periods, 1.63 us) echoed equal at 9600
  and at 115200, IRLP read back set (ten runs); LIN's detection: a 13-bit
  break raises LBD at both lengths, a 9-bit low at neither, a 10-bit low
  not at 11.
- **Half duplex hears a second chip** (letter `p`): HDSEL on PB0 with the
  pad an input - floating, and pulled up - receives the four 0x5A frames
  the far chip drives onto that wire from its own RX pad.
- **A pad handed to the function as TE rises shows a low pulse** (the
  far chip's USART, measured across this wire, the same block's
  lineage): the link's end parks its TX pad as a GPIO output high across
  a reconfiguration and hands it back two bit times and 20 us after the
  enable.
- **The receive vector's two guards** (letter `t`, ten runs), each staged
  by a memory-to-memory block on channel 5 at very_high - the ring's own
  level, and the lower number wins the tie - started once the far chip's
  burst of eight at 115200 had put four frames in the ring: the overrun
  with nothing to take, the vector held off until the block (4096 items,
  512 us) and the burst were over - ORE up, RXNE down, no frame after it:
  four entries, no channel given up, the overrun and the gap counted once
  each, five bytes delivered (the four before and the one DATAR held), the
  vector back waiting for the end; the stall bound, the vector running
  while a block of 65535 items starved the ring 8.2 ms - RXNE standing
  under RXNEIE: 257 entries (the overrun's, 255 of the wait for a frame,
  a tick's), the live channel given up once, a DMA fault counted, and the
  consumer's look binding it again. The next burst after each, whole and
  in order.
- **The idle line under the channel's clear, frame by frame** (letter
  `u`, ten runs): a one-shot channel 8 takes the far chip's groups 5 ms
  apart, STATR read once three frame times after each group. Groups of
  one frame read IDLE 1 0 1 0 1: ONE frame after an idle that was read -
  the status read saw IDLE and armed its clear - raises no IDLE of its
  own (its channel read performs the clear and forgets the idle its frame
  armed), and one after a status read that found IDLE down does: that
  read armed no clear of IDLE. Groups of two read IDLE every time.
- **A frame that completes between the status read and the data read is
  lost, its overrun cleared unseen**: the F1 clear sequence - STATR read,
  then DATAR - clears ORE whether it rose before the status read or after
  it. Measured at 3 Mbaud by `usart_peer`'s receive loop with its first
  frame's time stamp (a few dozen cycles) taken between the two reads:
  two gapless runs in five lost a frame and reported no overrun; with
  DATAR read first and the stamp after, none since. A polled receiver
  reads DATAR the moment STATR says RXNE.

### This part as the desk's USART instrument

`usart_peer` runs USART4 on PB0/PB1 for the CH32V006's suite
(`test_ch32_serial`'s two-board key), the far chip on its 24 MHz crystal
- the protocol's peer with what this die offers:

- **Calibrated by its DUT**: the HSI is the only root, so the peer
  starts as an RC peer and adopts the DUT's crystal through calibrate - a
  gapless run at 1 Mbaud, timed first frame to last on the STK - moving
  HSITRIM to the step nearest 48 MHz and keeping the rate it measures
  there for every divisor, every tick a report carries and every edge its
  engine places ([clock.md](clock.md) has the figures). Its blasts then
  state their rate: 48 MHz over 52 at 921600 read +1091 to +1556 ppm,
  the divisor's +1602 and the adopted residual.
- **Its echo** is 28 instructions a character with no call - a status
  load, a data load, the tally, the store into DATAR on TXE - fetched from
  the flash; the receive loop holds 3 Mbaud gapless (a gapless run timed
  by it within 200 ppm of the CH32V006's divisor at every rate from 9600
  to 3 Mbaud, against a 1 Mbaud run back to back).
- **Its waveforms** are TIM1's complementary outputs C2N and C3N on PB0
  and PB1 under remap 001, edge lists streamed by DMA on CH2's and CH3's
  requests ([tim.md](tim.md)), one pulse for a glitch, USART4 moved to its
  column 1 (pads left inputs) while the timer owns PB0: six banged 0x55
  frames clean at 9600, one stretched cell breaking the frame from +50
  per cent - where the CH32V006's own pull-banger on the same wire breaks
  it too - a 13-bit break ahead of a frame, glitches 6 to 2500 ticks
  wide, frames at 9600 off by -8 to +8 per cent.
- **The wired AND**: its collide pulls 0x3C into the CH32V006's 0xF0 as
  an emulated open drain on its RX pad, the far pad an open drain too,
  and its own receiver reads 0x30 off the wire.

## Not covered yet

Driver gaps, each with its reason:

- **The synchronous, smartcard, IrDA, mute and flow-control modes as
  tasks** (the AVR's `SyncHost`, `IrdaLink` and the like): the resource's
  verbs are the whole of it, and a task is born with its first user.

Implemented but not bench-verified, each with what would measure it:

- **What the other modes do on a wire**: the smartcard's guard time and
  NACK, the clock on CK and the sense of LBCL want a logic analyser on
  the pads or a peer on a CK pad this package does not bond, and the
  flow-control pair's handshake a peer on the CTS and RTS pads (PB15 and
  PA8, unbonded on the QFN20).
- **The stall bound over a frozen channel**: measured over a live one a
  block above the ring starved (letter `t`); a channel a request another
  block holds on its OR leaves the vector the same unmoved count - the
  CH32V006's suite measured that shape ([../ch32v00x/usart.md](../ch32v00x/usart.md)),
  and here channel 8's other requesters are TIM2's trigger and COM
  events, whose held request no letter stages yet.
- **USART1 and USART3, and the other packages' columns**: USART1 has no
  usable column on the QFN20 and USART3's only one is the debug port's,
  so neither has carried a byte here; a board of another package would
  measure them and the columns this one does not bring out.
