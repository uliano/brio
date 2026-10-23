# UART (RP2040)

This document is for a programmer who needs a serial port on an
RP2040: the console a program prints to, a line-oriented link, a byte
transport to another chip. The chip's two UARTs are ARM's PL011, so the
driver is written once in the IP stratum (`brio/pl011/`, a directory
named for a peripheral design rather than a chip) and this stratum
supplies the chip's part and the public names. `Pl011<n>` is the
instance, one function per register fact. `Uart<n, pins, ...>` is the
task over it (a task, in brio, is a driver layer with state of its own,
served from its peripheral's interrupt, that an active object or
`print` calls; an active object is a component of a brio program with
its own event queue, served one event at a time by the kernel,
[docs/learn/introduction.md](../learn/introduction.md)): a byte
transport with a transmit ring and a receive ring, with the same public
surface as the other targets' tasks, which is what lets
`util/serial_port.hpp` and `print()` run here unchanged
([design/serial.md](../design/serial.md) is the stack above it).

The smallest use, the console on the Debug Probe's UART bridge, from
"How to use it" below:

```cpp
constexpr brio::UartPins console_pins{
    .tx = {0, brio::PinFunction::uart},
    .rx = {1, brio::PinFunction::uart},
};
using Serial = brio::Uart<0, console_pins>;
constexpr Serial serial;

extern "C" void isr_uart0() { (void)Serial::isr(); }

int main() {
    SysClock::init();
    Serial::init(clock, 115200);
    brio::enable_interrupts();
    brio::print(serial, "hello", brio::crlf);
}
```

Line by line: `console_pins` names GP0 and GP1 under the UART function
(function 2), which table 279 gives UART0 as its transmit and receive
pads - a pin the table does not give the instance does not compile;
`Serial` names the transport type (`0` the instance, the pads, and
every other template argument at its default: a 64-byte receive ring,
a 256-byte transmit ring, no DMA engines, the code in the flash);
`serial` is an empty tag object of that type, the argument `print`
takes to pick the port, since the type's functions are all static;
the `extern "C"` function is the interrupt handler, as in C, under the
pico-sdk's name for UART0's line; `init()` takes the clock task's tag
(`clock`, [clock.md](clock.md)) and the baud rate; `brio::crlf` is the
line ending.

What decides correctness even there. `init()` after the clock's: the
divisor is computed from the rate the clock type claims for clk_peri
(`pclk_hz`) and not read from the hardware, and clk_peri itself is off
until `SysClock::init()` enables it. The vector is the program's to
bind: the crt gives `isr_uart0` a weak alias to `Default_Handler`'s
spin, so a port whose vector is left unbound parks the core there at its
first interrupt - the first received byte, or the first transmit that
backs up behind a full FIFO; and the name `isr_uart0` is a macro of the
vendored device header for the vector's slot, so the handler is defined
in a file that includes a brio header ([platform.md](platform.md),
"Bench findings"). `isr()` returns true when a received byte made the
receive ring non-empty, the edge an active object is told of: the
smallest use discards it, and the kernel's console posts it to the
active object that assembles lines (the second block under "How to use
it"). Three more things to know before the first `print`: `print` waits
until the transmit ring has taken every byte, so a long message printed
from an active object's handler holds every other active object for that
long ([design/serial.md](../design/serial.md)); the transmit ring has
one producer, so `write_byte` and `write_bulk` are called from active
objects and from `main`, never from an interrupt handler, and both copy,
the caller's buffer being free when the call returns; and an idle
transmitter takes the first 32 bytes straight into its FIFO with no
interrupt, so a short message leaves before the first transmit interrupt
ever fires ("How the transmitter starts", below).

The page is long because the chapter is. In order: the documents of
record; what the silicon does; the types and verbs, with how the
transmitter starts; how to use it; the bench findings - the loop-back,
the pair's wires against the RP2350B, the engines, the probe's bridge,
the receive and transmit costs, the vendor's loops on the same board;
and what is not covered.

## Documents of record

The RP2040 datasheet (build 3184e62), 4.2
(UART: the ARM PL011 - 4.2.2 functional description, 4.2.3 operation,
4.2.6 interrupts, 4.2.7 the programmer's model and the baud
calculation of 4.2.7.1, 4.2.8 the registers), 2.19.2 (table 279: the
pins), 2.15.3.1 (clk_peri). The driver is ARM's and not this chip's, so
it lives in the IP stratum: `brio/pl011/uart.hpp` holds the resource and
the byte transport ([../pl011/README.md](../pl011/README.md)), and
`brio/rp2040/uart.hpp` holds what this chip owes it - the pin table of
table 279, the `Rp2040Pl011` chip traits over `pin.hpp`, `resets.hpp`,
`clock.hpp`, `nvic.hpp` and `dma_engine.hpp`, and the public names
`Pl011<n>` (the resource) and `Uart<n, pins, ...>` (the task), this
chip's aliases of the two templates there. The reference suite:
`test_rp2040_serial` (the second instance under the loop-back for the
formats, the ladder, the FIFOs, a break, an overrun and bulk traffic;
the desk partner's `uart_peer` on the pair's UART wires; the host at the
console's other end); a suite's letter is one command key of its
console and the group of verdicts it runs, and the desk partner is the
board wired to the WeAct RP2040 the suite runs on, a WeAct RP2350B
([../boards/weact-rp2350b.md](../boards/weact-rp2350b.md); the wires
between the two are listed on
[../boards/weact-rp2040.md](../boards/weact-rp2040.md)).

## What the silicon does

Two ARM PL011s, clocked from clk_peri (UARTCLK) with their registers
on clk_sys. The baud rate divisor is UARTCLK / (16 x baud), a 16-bit
integer (UARTIBRD) plus a 6-bit fraction (UARTFBRD), so the reach is
UARTCLK/16 down to UARTCLK/(16 x 65535); the two registers take
effect on the next write of the line control register, and LCR_H,
IBRD and FBRD are written with the UART disabled. Both FIFOs are 32
deep; the receive FIFO stores the framing, parity and break flags
with each byte (bits 8..10 of UARTDR), while the overrun flag (bit
11) is a live condition - set while the FIFO is full and a frame
lands, cleared as soon as a read makes room - so the entries never
show it and UARTRSR's sticky OE is where an overrun is read
(measured). The loop-back bit UARTCR.LBE feeds the transmitter into
the receiver with the RX pad ignored. Eleven maskable interrupts
combine into one line per instance (UART0_IRQ, UART1_IRQ): receive at
the FIFO's trigger level, receive timeout when bytes wait and the
line has idled 32 bit periods, transmit when the FIFO falls through
its trigger level - "based on a transition through a level, rather
than on the level itself" (4.2.6.3), cleared by writing the FIFO
above the level or through UARTICR, so enabling it over an empty FIFO
raises nothing - and the four errors. The
modem inputs and DTR/OUT1/OUT2 exist in the block and reach no pad on
this chip; CTS and RTS do. The pins are fixed per instance under
function 2: UART0 transmits on GPIO 0, 12, 16, 28 and receives on 1,
13, 17, 29; UART1 on 4, 8, 20, 24 and 5, 9, 21, 25.

## Types and verbs

- `UartBits` (5..8), `UartParity`, `UartFormat` (bits, parity, 1 or 2
  stop bits), `uart_format_valid`.
- `UartPins` (`PinSel` tx and rx), `uart_tx_pin(n, pin)`,
  `uart_rx_pin(n, pin)`, `uart_pins_valid(n, pins)` - table 279 as
  constexpr facts; a pin set the table does not give the instance
  does not compile.
- `UartDivisor` (integer, fraction), `uart_divisor(hz, baud)` (nothing
  when out of reach), `uart_actual_baud(hz, d)`, `uart_min_hz(baud)`.
- `UartFlag` (UARTFR), `UartInterrupt` (one layout for IMSC/RIS/MIS/
  ICR), `UartDataError` (UARTDR's flags), `UartReceiveStatus`
  (UARTRSR's sticky four), `UartControl` (UARTCR), `UartLineControl`
  (UARTLCR_H), `UartTriggerField` (UARTIFLS), `UartBaudField` (the two
  divisor registers' masks), `UartDmaControl` (UARTDMACR),
  `UartFifoLevel` (the trigger levels in eighths).
- `Pl011<n>` - the resource: `reset`/`hold`/`released` (through the
  reset controller), `enable(on)` (UARTEN with TXE and RXE),
  `line_control(format, fifos)`, `divisor(d)` and `loopback(on)`
  (all three refused while enabled; the divisor followed by the
  latching LCR_H write), `break_send(on)` (live), `fifo_levels(rx,
  tx)` and their readback, `fifos_enabled`, the flags (`tx_full`,
  `rx_empty`, `busy`), `read_data`/`write_data`, the receive status,
  the interrupt mask/pending/clear trio through the atomic aliases,
  `dma_requests`, `irq()`.
- `Uart<n, pins, rx_size = 64, tx_size = 256, TxEngine, RxEngine, Placement>` -
  the task, every target's Uart surface: `init(clock, baud, format,
  rx)` (`rx` the PL011's own: `UartRx::pad`, or `UartRx::loopback` for a
  loop-back from the first sample), `isr()` (true on the receive ring's
  empty-to-non-empty edge), `write_byte`, `write_bulk`, `read_byte`,
  `read_bulk`, `read_span` and `consume` (the receive run in place),
  `rx_pending`, `tx_idle`, `rebase(hz)`, `set_baud(hz, baud)`,
  `set_format(format)`, `loopback(on)` (each under the running port,
  after a drain), `min_hz_for`, `can_baud`, `actual_baud(hz)`,
  `release()`; the counters `rx_overruns` (ring), `frame_errors`,
  `parity_errors`, `break_errors` (a break entry carries FE too, and
  both are counted), `hw_overruns` (one per overrun event: UARTRSR's
  sticky OE on the interrupt receiver, the OE interrupt under an
  engine - UARTRSR cleared and the source masked until the next publish,
  [../pl011/README.md](../pl011/README.md)), `clear_errors`; each counter
  holds at its top rather than wrapping. The receive engine arms at
  HIGH_PRIORITY and the transmit engine at normal, and the Uart takes no
  level option ([dma.md](dma.md)). The two engine slots take `dma.hpp`'s
  `DmaTxEngine` / `DmaRxEngine` ([dma.md](dma.md)): with a transmit
  engine the ring's runs leave as DMA blocks and `dma_isr()` (the
  line's ISR body) releases each and starts the next - a block start
  masks the engine's claim alone (six instructions, about 10 cycles,
  counted in the listing) and programs the channel with the mask down,
  two stores to the DMA block a block; with a receive
  engine the run is filled straight from UARTDR, a run that fills is
  published and re-armed by its completion on the line - `dma_isr()`
  answering the ring's edge - and a run still filling by `harvest()`,
  which publishes what TRANS_COUNT says has landed and re-arms; the
  error interrupts count each received error (the engine moves an
  entry's byte, a break's zero included, and not its flags), and no
  vector ends a burst under an engine
  ([../pl011/README.md](../pl011/README.md), "The receive side");
  `dma_faults()` counts the blocks abandoned after a bus error.
- `Placement`, the last parameter: util/placement.hpp's tag, `InFlash`
  by default, `InRam` making the bodies of `isr()` and `dma_isr()` one
  flattened instance each in SRAM ([platform.md](platform.md), "A
  handler in SRAM").

HOW THE TRANSMITTER STARTS, because the interrupt is an edge that
stays latched (the IP stratum's page, [../pl011/README.md](../pl011/README.md),
where the rule and its measurement on the RP2350's same block are):
a byte with nothing queued ahead of it and room in the FIFO is
written straight into UARTDR by `write_byte` (a run at a time by
`write_bulk`), so an idle transmitter takes 32 bytes with no
interrupt; a byte behind a full FIFO is queued in the ring and arms
the transmit interrupt, whose handler refills the FIFO once per fall
through its level and disarms it when the ring runs dry; a byte the
full ring refuses writes nothing. The handler is the ring's one
consumer, and the loop writes the data register only while the ring
is empty.

## How to use it

```cpp
constexpr brio::UartPins console_pins{
    .tx = {0, brio::PinFunction::uart},
    .rx = {1, brio::PinFunction::uart},
};
using Serial = brio::Uart<0, console_pins>;
constexpr Serial serial;

extern "C" void isr_uart0() { (void)Serial::isr(); }

int main() {
    SysClock::init();
    Serial::init(clock, 115200);
    brio::enable_interrupts();
    brio::print(serial, "hello", brio::crlf);
}
```

With the kernel, the edge feeds `SerialPort`:

```cpp
extern "C" void isr_uart0() {
    if (Serial::isr()) { brio::post<SerialLines>(brio::RxActivity{}); }
}
```

## Bench findings

- The divisor from `Clock::pclk_hz` at 125 MHz for 115200 is 67 +
  52/64, the datasheet's own worked example (115207 baud, 0.006 %),
  read back in UARTIBRD/UARTFBRD.
- The reference suite, green on both boards, on the second instance:
  - every frame format - 5..8 bits, none/even/odd, 1 or 2 stops -
    byte-exact on the loop-back at 115200;
  - the ladder from the floor to the ceiling at clk_peri 125 MHz:
    120 baud (65104 + 11/64), 300, 9600, 115200, 921600, 2 M, 4 M and
    7812500 (1 + 0/64), each byte-exact and taking its frames' time;
    119 and 7.9 M refused, and a rate within half a sixty-fourth of
    a divisor rounds onto it (7812501 is 7812500);
  - THE RECEIVE TIMEOUT AT EVERY RUNG (letter c, the line masked and
    UARTRIS.RTRIS polled): a lone byte raises it 41.5 bit periods after
    its write - its stop bit's middle, where the byte enters the FIFO,
    and 32 bit periods - at 120, 300, 9600, 115200, 921600, 2 M, 4 M and
    7.8125 Mbaud alike (347.8 ms at 120 baud against 348.7, 363 us at
    115200 against 360, 7 us at the ceiling against 5); the line
    unmasked, the transport's vector hands the byte to the program 7 to
    19 us later (eighty bytes, ten runs);
  - `rebase` UNDER A RUNNING PORT (letter r): clk_peri moved from clk_sys
    to the crystal under both UARTs, each rebased to its baud - the
    console printing on, the loop byte-exact, 1024 frames in their wire
    time at 115107 baud (6 + 33/64 of 12 MHz) - and back; `set_baud`
    under a running port is how letter c climbs its ladder, every rung
    exact and at the rate asked;
  - THE TWO DELIVERIES: a lone byte reaches the ring 371 us after
    its write at 115200 - one frame plus the 32-bit receive timeout;
    a burst of sixteen is delivered by the level, its first byte
    only when the sixteenth has landed (1401 us), which is why a run
    shorter than a level costs the timeout at the end (267 ms at 120
    baud);
  - a 1 ms break counts one BE (and one FE, on the same entry) and
    delivers no byte; 48 frames into the 32-deep FIFO with the line
    masked count one OE - from UARTRSR, as the entries never carry it -
    and the look after them skips what the ring held: nothing handed
    out, one skip, the four bytes after it whole;
  - 4096 bytes round the loop at 3 Mbaud byte-exact in 13.7 ms (the
    wire's own 13.65);
  - THE RATE ASKED, which a loop cannot tell (both ends share one
    divisor), timed on the ruler from the first write to `tx_idle()`:
    every rung's frames in 1000 to 1003 thousandths of their wire time
    from 120 baud to 4 Mbaud and 1011 at the 7.8125 Mbaud ceiling, where
    the loop's receive levels share the one line with the refills
    (`bench_rp2040`'s print, transmit alone, runs at x 1.00 there);
    256 frames of every format in 1000 to 1001 - the parity and stop
    bits really sent; at an exact 9600 the 64 frames of the ladder came
    back in 66664 us of the 66666 they occupy, the receiver having the
    last byte before its stop bit ends, which the rung's lower bound
    allows.
- ON THE PAIR'S WIRES, against the WeAct RP2350B's `uart_peer` (UART1
  crossed, the peer's role chosen on GP21 into its mode line,
  [../boards/weact-rp2040.md](../boards/weact-rp2040.md)), the suite's
  letters p and s at 115200 8N1, each end on its own crystal:
  - 512 bytes to the peer's echo and back byte-exact, no error, in 45861
    to 45865 us against the 45830 derived - 528 frames at 115207 baud, the
    peer echoing a run of sixteen (its receive level) behind this end -
    with the peer on either RP2350 architecture: the two crystals agree
    on the rate, which no loop-back can show;
  - listening at 7E1 to the peer's 64 pattern frames at 8N1: a parity
    error counted, and the frame dropped, exactly where the byte has an
    odd number of ones (30 of 64), no frame error - and nothing handed
    out, since every run of sixteen the handler moved held an error and
    the ring skipped each whole (four skips, the receive ring's contract,
    [../pl011/README.md](../pl011/README.md));
  - the other way round, 64 frames sent at 7E1 into the peer's 8N1 echo
    back exact with no error, in 6972 to 6974 us against the 6944 of 80
    frames: a 7E1 frame is an 8N1 frame whose eighth bit is the parity.
- THE LOOP-BACK IS ASKED FOR AT THE BRING-UP in every suite letter on
  it: with the far board dark, a 3 Mbaud receive engine brought up on the
  pad and switched to the loop-back after took a byte nobody sent at the
  head of its run in three bring-ups of five, and none in twelve once the
  loop-back was chosen at `init()`; and under the loop-back the GP4 pad
  still carries every frame (eight start bits for eight frames) - the
  IP's findings ([../pl011/README.md](../pl011/README.md)).
- THE ENGINES (`test_rp2040_dma`): 4096 bytes through the loop-back
  at 3 Mbaud with an engine in each slot, byte-exact, 22 interrupts;
  the two traps of a receive engine - a break taken at enable over a
  low pad, and the request credits that outlive a completed run - are
  the transport's business now ([dma.md](dma.md)).
- THE OVERRUN UNDER A RECEIVE ENGINE (`test_rp2040_dma` letter o, three
  episodes of 5120 frames at 3 Mbaud into a 1024-byte ring nobody
  reads): with the OE interrupt cleared through UARTICR alone, ONE
  entry and one count for the three - UARTRSR's sticky OE left standing
  hid the second and the third; with UARTRSR cleared too, 4058 to 4065
  entries an episode, one a frame lost; with the source masked from the
  loss to the next publish, one entry and one count an episode and the
  vector quiet in the 10 ms after each
  ([../pl011/README.md](../pl011/README.md)).
- THROUGH THE DEBUG PROBE'S BRIDGE, the console's own ladder fed by
  the host: byte-exact at 115200, 460800, 921600, 1 M, 2 M and 3 M
  baud (142 KB at 3 M with no error and no overrun, host to board),
  and board to host - a quarter of a second of line at each rung, 2880
  bytes at 115200 to 75 000 at 3 M, the host judging - every byte
  arrived exact;
  the bridge sends 8N1 whatever parity the host asks for, so a
  parity-error leg through it is a measure of the probe, not of the
  UART ([../probes/raspberry-pi-debug-probe.md](../probes/raspberry-pi-debug-probe.md)).
- The transport works on the wire in both directions through the
  Debug Probe's bridge at 115200: the pin-check tool's banner and its
  echo-with-code of every character typed, the kernel console's
  banner, replies and prompt through `SerialPort` and `print`, with
  the frame, parity, break and hardware-overrun counters and the ring
  overrun all at zero. The receive timeout interrupt is what
  delivers a single typed character - and it is also what exposed
  the crt's handler-name trap ([platform.md](platform.md)) before the
  first byte.

- THE TRANSMIT POLICY MEASURED ON THE WEACT RP2040 as on the RP2350's
  same PL011 r1p5 ([../rp2350/uart.md](../rp2350/uart.md)): the direct
  write into an idle FIFO, the transmit interrupt armed only behind a
  full FIFO - `test_rp2040_serial` 32 pass whole, a 4096-byte print 501
  interrupts at x 0.99 of the wire and 256 bytes 31 (`bench_rp2040`
  letter p).

- THE RECEIVE SIDE AND ITS EDGE (`bench_rp2040` letter u, UART1 under
  LBE at 125 MHz, the meter's stamps inside `isr`): 256 bytes through the
  interrupt receiver in 16 level entries and the time-out's, 12882
  cycles at 115200 and 11659 at 1 Mbaud - 46 a byte, moving by up to 4
  a byte between two runs of one image with the XIP cache - where the level
  read with a flag test before every character took 16510 and 15500 (60
  a byte); the receive engine 390 cycles for the same 256, two
  completions. The edge from the burst's last stop bit (BUSY falling):
  the time-out's 3.2 frames at 115200, and at 1 Mbaud 1.6 to 2.7 frames
  from run to run (measured, not dissected); under the engine
  `UARTRIS.RTRIS` never rose (17 bytes and ten frames of silence: the
  FIFO empty, every byte moved), so the engine's burst edge is its
  owner's poll - 2.4 frames at 115200 to 43 at 1 Mbaud on a 1 ms tick.
- THE TRANSMIT SIDE through the same letter: 4096 bytes in 145 refills
  at 25 cycles a byte (27 before), wire-bound (x 1.00) to UARTCLK / 16 =
  7.8125 Mbaud; through the transmit engine in 40 to 47 blocks, about 3
  cycles a byte, wire-bound too.
- `tx_idle()` ON THE PAD (`test_rp2040_serial` letter t, eight frames of
  0xFF at 9600 on GP4 with the loop-back off, the pad read through SIO):
  true 0 us from the last stop bit's end, a bit being 104 us; under the
  transmit engine (`test_rp2040_dma` letter t) the same.
- THE COPY INTO THE RING: the byte loop 2 cycles and 9 a byte, the
  runtime's `memcpy` about 54 and under one a byte with the two ends
  aligned alike - the loop ahead at 8 bytes, `memcpy` at 12, the
  family's `copy_crossover`.
- AGAINST THE VENDOR: the pico-sdk 2.3.1 has no interrupt or DMA
  receive, only `uart_read_blocking` and `uart_write_blocking`, POLLED
  loops (a flag test and a data access a byte), run on the same board
  and loop in a scratch program: 58 cycles a byte reading characters
  already in the FIFO and 53 writing into an empty one, every cycle of
  the wire's time the CPU's. This transport's interrupt receiver costs
  46 a byte and its transmitter 25, and the core is free between
  entries.

## Not covered yet

Driver gaps, each with its reason:

- Hardware flow control (CTS/RTS, 4.2.4): two more wires on the
  cross link (GP6/GP7 are UART1's CTS/RTS); the pads are table 279's
  next two of each group.
- IrDA (SIR, the low-power counter): an IrDA transceiver.
- The stick-parity bit (SPS): born with a first protocol that uses
  it (a nine-bit address mark).
- The FIFO trigger levels other than the task's half and eighth: a
  program with a reason to move them; the resource writes any pair.

Implemented but not bench-verified, each with what would measure it:

- Why a burst of 17 at 1 Mbaud is told 1.6 to 2.7 frames after its last
  stop bit, under the 32 bit periods of the time-out (letter u; the
  M33 and Hazard3 read 3.26 on the same source) - where a lone byte's
  time-out is 32 bit periods from its stop bit at every rung (letter
  c), so the shortfall is the burst's, not the rate's: the entries timed
  against the RX pad would place each one.
- The 7 to 19 us from a masked time-out's line unmasked to the byte in
  the program's hands (letter c), some 900 to 2400 cycles where the
  receive path costs 46 a byte in a burst: the vector's entry and the
  copy timed warm and cold on the meter (`bench_rp2040` letter u's
  instrument) would split it.
