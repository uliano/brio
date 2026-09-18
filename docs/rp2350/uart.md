# UART (RP2350)

Documents of record: the RP2350 datasheet (build d126e9e), 12.1 (UART:
two ARM PL011s, revision r1p5 - the FIFO depths and the feature list in
its opening, the programmer's model and the registers below it), 9.4 (the
GPIO function table, which is where this chip differs most from its
predecessor), 12.6.4.1 (the system DREQ table), 7.5 (the reset
controller), 3.2 (the interrupt lines), 2.1.3 (the atomic register
aliases). The driver is ARM's and not this chip's, so it lives in the IP
stratum: `brio/pl011/uart.hpp` holds the resource and the byte transport
([../pl011/README.md](../pl011/README.md)), and `brio/rp2350/uart.hpp`
holds what this chip owes it - the pin table with BOTH of its function
columns, the `Rp2350Pl011` chip traits over `pin.hpp`, `resets.hpp`,
`clock.hpp`, `core.hpp` and `dma_engine.hpp`, and the PUBLIC NAMES
`Pl011<n>` (the resource) and `Uart<n, pins, ...>` (the task), this
chip's aliases of the two templates there. The reference suite:
`test_rp2350_serial`, which runs on BOTH of this chip's architectures
from one source (the second instance under the loop-back for the formats,
the ladder, the FIFOs, a break, an overrun and bulk traffic; the second
function column on its own pads; a peer board on the cross link; the host
at the console's other end).

This page says what is the RP2350's. What the block itself does - the
fractional divider, the disabled-write rule, the transmit interrupt that
is a transition, the per-entry error flags, the engine slots - is the IP
stratum's page and is not restated here.

## What the silicon does

The block is the same one the RP2040 carries, at the same revision and
with the same 32-deep FIFOs (12.1: 32x8 transmit, 32x12 receive), clocked
from clk_peri as UARTCLK with its registers on clk_sys. Four things about
it are this chip's:

**TWO FUNCTION COLUMNS CARRY UART DATA.** The pads march in groups of
four - TX, RX, CTS, RTS - and the groups cycle UART0, UART1, UART1,
UART0 and again, over the whole bank: GP0..GP3 are UART0's, GP4..GP7
UART1's, GP8..GP11 UART1's, GP12..GP15 UART0's, up to GP47 on the QFN-80.
That much is the RP2040's rule over a longer bank. What is new is that
EVERY GROUP'S FLOW-CONTROL PADS CARRY DATA TOO, under a second code: the
CTS pad is also that instance's transmitter and the RTS pad also its
receiver, under function 11 where the first two pads use function 2. So
GP2/GP3 are a second UART0 pair, GP6/GP7 a second UART1 pair, and a
QFN-80 part offers twelve transmit pads and twelve receive pads per
instance against the RP2040's four. Which column a pad uses is not a
choice: a pad carries a UART signal under exactly one code, and the pad
number says which.

**THE PADS COME UP ISOLATED** (9.11, and [README.md](README.md)): a pad
answers nothing until the isolation latch is cleared, which every
configuring verb of `pin.hpp` does as it writes. The transport's receive
pad is handed over with a PULL-UP, because this chip's pads reset pulled
DOWN and a receiver enabled over a low line takes a break.

**THE DREQ NUMBERS ARE NOT THE RP2040'S** - not one row of the table is.
A third PIO and twelve PWM slices against eight move everything above the
first PIO's rows: the UARTs sit at 28..31 where they sat at 20..23.
`rp2350/dma_engine.hpp` carries the table so that a transport can name a
request without including a controller, and the engines that take those
requests are [dma.md](dma.md)'s.

**THE INTERRUPT NUMBERING IS SHARED BETWEEN THE ARCHITECTURES** (3.8.4.2),
so UART0_IRQ is line 33 and UART1_IRQ line 34 whichever processor pair is
running, and an app binds `isr_uart0` / `isr_uart1` once for both. The
controller behind those names is not the same object - an NVIC on the
Cortex-M33 half, Hazard3's own on the other - which is exactly why the IP
stratum's concept takes the controller as a TYPE.

## Types and verbs

What the IP stratum names (the frame vocabulary, the divisor arithmetic,
the register bit layouts, the resource's and the transport's verbs) is in
[../pl011/README.md](../pl011/README.md). This stratum adds:

- `UartPins` - a `PinSel` for each direction, each carrying the pad
  number AND the function that routes a UART to it.
- `uart_pad_instance(pin)` - which instance the four pads of that pin's
  group belong to.
- `uart_pad_function(pin)` - WHICH column routes a UART signal to that
  pad: `PinFunction::uart` (2) on the first two pads of a group,
  `PinFunction::uart_alt` (11) on the other two.
- `uart_tx_pin(n, pin)`, `uart_rx_pin(n, pin)` - the table of 9.4 as
  constexpr facts, over both columns.
- `uart_pins_valid(n, pins)` - a legal pin set for instance n: a
  transmit pad and a receive pad of that instance, EACH NAMED UNDER ITS
  OWN COLUMN. Naming GP6 under function 2 asks for UART1's CTS and not
  its transmitter, and does not compile. The PACKAGE is not asked here:
  the table is the die's, and a pad the QFN-60 has not bonded is refused
  one level down by `Pin<n>`'s own static_assert, which says so in those
  words.
- `UartColumn` and `UartPad<pin>` - what `pad_function` IS on this chip.
  The IP file calls it "the function code that routes a UART to a pad"
  and keeps it opaque; here the code is a property of the PAD, so the
  chip's answer is a tag meaning "the column that carries a UART signal"
  and the pad type turns it into the one code that routes a UART there.
  The IP file needed no change for it.
- `Rp2350Pl011` - the chip traits, which nothing above this file names.
- `Pl011<n>` and `Uart<n, pins, rx_size = 64, tx_size = 256, TxEngine,
  RxEngine>` - the public names, this chip's aliases of the two IP
  templates, with `NoDmaEngine` in both slots by default.

## How to use it

The console, on the first column:

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

The same source builds for both architectures; `isr_uart0` is a
vector-table slot on one half and a dispatch entry on the other.

The SECOND COLUMN is named and nothing else changes - the pads that carry
UART1's CTS and RTS under function 2 are its transmitter and receiver
under function 11:

```cpp
constexpr brio::UartPins alt_pins{
    .tx = {6, brio::PinFunction::uart_alt},
    .rx = {7, brio::PinFunction::uart_alt},
};
using Link = brio::Uart<1, alt_pins>;
```

With the kernel, the edge feeds `SerialPort`:

```cpp
extern "C" void isr_uart0() {
    if (Serial::isr()) { brio::post<SerialLines>(brio::RxActivity{}); }
}
```

## Bench findings

Everything below is `test_rp2350_serial` on the bench part (a QFN-80
stepping A2 at clk_sys and clk_peri 150 MHz), and every one of it is the
same on both architectures unless a number is given for each.

- THE DIVISOR at clk_peri 150 MHz for 115200 is 81 + 24/64, read back in
  UARTIBRD/UARTFBRD, which is 115207 baud - 60 ppm of the rate asked
  for. The reach runs from 144 baud (65104 + 11/64) to 9375000
  (clk_peri / 16, divisor 1 + 0/64); 143 and 9500000 are refused, and the
  generator's own reach runs to 9448818 before the rounding would make
  the integer part zero.
- SEVERAL STANDARD RATES ARE EXACT AT THIS CLOCK, which the RP2040's
  125 MHz did not make them: 9600, 1 M, 2 M, 3 M and 4 Mbaud all divide
  150 MHz with nothing left over. An exact rate leaves none of the
  rounding margin an inexact one hides a fraction of a bit in, which is
  why the suite's lower time bound allows the one bit period a receiver
  does not wait out (at 9600, 64 frames took 66661 us against the 66666
  the frames nominally occupy).
- Every frame format - 5..8 bits, none/even/odd parity, 1 or 2 stop bits,
  twenty-four of them - byte-exact on the loop-back at 115200, no error
  flagged.
- THE LADDER on the loop-back, each rung byte-exact and taking its
  frames' time: 144 baud (143 real), 300, 9600, 115200 (115207), 921600
  (921658), 2 M, 4 M and 9375000.
- THE TWO DELIVERIES: a lone byte reaches the ring 367 us after its write
  at 115207 baud - one frame (87 us) plus the 32-bit receive timeout
  (278 us); a burst of sixteen is delivered by the LEVEL, its first byte
  only when the sixteenth has landed (1396 us), all sixteen within
  1402 us.
- A 1 ms break counts one BE (and one FE, on the same entry) and delivers
  no byte. Forty-eight frames into the 32-deep FIFO with the line masked
  deliver exactly 32 in order and count one OE.
- BULK TRAFFIC, 4096 bytes round the loop, byte-exact with nothing
  overrun at every rung tried - 460800, 921600, 1 M, 2 M and 3 Mbaud -
  with the code running out of the flash through the interface the
  bootrom leaves set up (03h serial reads at CLKDIV 12; there is no
  second-stage bootloader on this chip and no chapter reprograms the QMI
  yet). At 3 Mbaud: 13675 us and 1044 interrupts on the Cortex-M33 half,
  13686 us and 978 on the Hazard3 half - 25 and 23 interrupts per hundred
  bytes, against the wire's own 13653 us. On a LOOP-BACK the receive FIFO
  is fed at exactly the rate the transmit FIFO empties, so neither runs
  far ahead of its trigger level and the batching is nothing like the
  FIFO's depth.
- THE SECOND FUNCTION COLUMN, with UART1 moved onto GP6/GP7: both pads
  read back FUNCSEL 11 and out of isolation, and THE TRANSMITTER REALLY
  REACHES THE PAD - read through the bank, which always reads the pad
  whoever owns it, the line idles HIGH, a break pulls it LOW and clearing
  the break lets it back up. The block is the same block there: 256 bytes
  round the loop byte-exact in 22254 us.
- ON THE CROSS LINK to a peer board (this board's GP4 to the peer's
  receiver and the peer's transmitter to this board's GP5, with GP21 as a
  mode line into the peer): 512 bytes to the peer's echo and back
  byte-exact in 45969 us on the Cortex-M33 half and 45884 us on the
  Hazard3 half, no error counted. Listening at 7E1 to the peer's 64
  frames at 8N1, the eighth data bit lands where the parity bit is
  expected - 30 parity errors counted and dropped, 34 frames accepted, no
  frame error - the attribution per entry, on a real wire. The peer is
  the `uart_peer` firmware, which has no console of its own and takes its
  role from that wire.
- THROUGH THE DEBUG PROBE'S BRIDGE, the console's own ladder fed by the
  host: byte-exact at 115200, 460800, 921600, 1 M, 2 M and 3 Mbaud, with
  no frame error, no hardware overrun and no ring overrun at any rung
  (144000 bytes at 3 Mbaud host to board on the Cortex-M33 half, 142500
  on the Hazard3 half). The bridge sends 8N1 whatever parity the host
  asks for, so a parity-error leg through it measures the probe and not
  the UART
  ([../probes/raspberry-pi-debug-probe.md](../probes/raspberry-pi-debug-probe.md)).
- The transport carries the kernel console and the pin-check tool on both
  architectures at 115200 through that bridge, with every error counter
  and the ring overrun at zero.

## Not covered yet

Driver gaps, each with its reason. The ones that belong to the block
rather than to this chip - hardware flow control, IrDA, the modem status
signals, the stick-parity bit - are in
[../pl011/README.md](../pl011/README.md) and are not repeated here.

- Hardware flow control has a second cost here that it has nowhere else:
  claiming a group's CTS and RTS pads for flow control is claiming the
  same two pads that are the instance's SECOND data pair, so a program
  cannot have both on one group.

Implemented but not bench-verified, each with what would measure it:

- THE TWO DMA ENGINE SLOTS, now that there is a controller to fill them
  ([dma.md](dma.md)): `test_rp2350_dma`'s letter j puts an engine in each
  slot of the second instance under the loop-back, and its letter k runs
  the console's own transmitter on one.
- THE RECEIVE HALF OF THE SECOND COLUMN. The transmitter is proven on its
  alternate pad by watching the pad itself; the receiver on an alternate
  pad is proven only as far as the pad register - no wire of this bench
  joins an alternate transmit pad to an alternate receive pad of the same
  instance. A wire from GP6 to GP7 (or any such pair) and the suite's
  letter f would close it.
- The alternate column on UART0 (GP2/GP3 and the rest): the table is one
  rule over both instances and letter f exercises UART1's; UART0 is the
  console on every image of this target, so its alternate pads want a
  program that gives the console up.
- The QFN-60's pin table: the stratum compiles for that package and
  refuses the pads it has not got, but no QFN-60 part is on the bench.
- `rebase` under a dynamic clock: this target has no dynamic clock yet,
  so the verb is exercised only at a rate that does not change.
- The bridge's ceiling board to host: the suite measures host to board.
