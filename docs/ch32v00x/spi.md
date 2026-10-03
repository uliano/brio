# SPI (CH32V00x)

The one SPI of RM ch. 16 - both roles, 8- or 16-bit frames, the four
modes, a software or hardware chip select, hardware CRC, two DMA
requests - and the host engine util/spi_bus.hpp's arbiter drives on
this silicon as on the other three. Documents of record: the CH32V00X
reference manual V1.5 (16.2.1 for the pins and the select, 16.2.2 and
16.2.3 for the two roles, 16.2.5 for the CRC, 16.2.7 for the errors,
16.3 for the registers), the CH32V006 datasheet V2.0 (table 2-1-1 for
the pads).

## What the silicon does

- **The STM32F1's SPI**, with the I2S half absent and one register of
  its own: HSCR, a "high-speed read" mode whose SCK the manual gives
  as HCLK/(BR + 2).
- **No FIFO.** One transmit buffer, one receive buffer, one shift
  register (figure 16-1): TXE means the buffer moved into the shifter,
  RXNE that a frame has been shifted both ways. A frame written while
  another shifts waits in the buffer and follows it with no gap on SCK
  (16.2.2: "if the TXE flag is set, fill the data register, maintain
  the complete data flow"); and the receive buffer holds ONE frame - a
  frame that ends while the one before it stands unread is lost, OVR
  set, the transmit side untouched (16.2.7), the flag cleared by a
  DATAR read followed by a STATR read. Those two sentences are what the
  host's loops and its write-ahead rest on, and what bounds them.
- **Frames of 8 or 16 bits** (DFF), SCK from HCLK/2 to HCLK/256 (BR),
  MSB or LSB first, the four CPOL/CPHA modes of table 16-1.
- **The chip select has three arrangements** (16.2.1): software (SSM,
  the level from SSI, the pad free), hardware output (SSOE, driven
  low while SPE is set), hardware input (the multi-host arrangement:
  a low level raises MODF and demotes the host).
- **NOTHING IS ENABLE-PROTECTED** (measured): 16.3.1 says DFF and CRCEN
  "can only be written when SPE is 0" and that BR, CPOL, CPHA, MSTR
  and LSBFIRST "cannot be modified during communication" - and a raw
  write to each of the seven lands with SPE set. The driver's
  refusals and its disable/enable pair are the only protection there
  is.
- **A DMA request that rises while its channel is disabled is
  LATCHED** and served at the channel's next enable (measured: with
  RXDMAEN standing through a command phase pumped by hand, the
  command's echo landed as the block's first byte; with it standing
  between transactions, the stale frame in DATAR did - every engined
  block after the first came back shifted by one). The host raises
  RXDMAEN only once the receive channel is enabled, TXDMAEN only once
  the transmit one is, and drops both at the block's end.
- **The DMA requests are channels 2 (receive) and 3 (transmit)**,
  table 8-2 - the channel is the request on this family. DATAR is
  sixteen bits, so a channel can move a 16-bit frame in one half-word
  beat - from a half-word boundary only: 8.3.6, a 16-bit access ignores
  the address's bit 0. Every frame that comes back was clocked out
  first, so the receive block ends after the transmit block: the
  receive channel's completion is the one edge a transaction needs.
- **The default pads on the CH32V006** are SCK PC5, MOSI PC6, MISO PC7,
  NSS PC1 (table 2-1-1); the remaps are AFIO's, which the stratum does
  not touch yet.
- **Two of the chapter's sentences are slips**: DFF's two codes are
  both described as "16-bit" (0 is 8-bit), and 16.2.1 describes CPHA
  with the same words for both values (table 16-1 and the figures
  settle it).

## Types and verbs

[brio/ch32v00x/spi.hpp](../../brio/ch32v00x/spi.hpp), three layers:

- `Spi<1>` is the resource: `configure(SpiConfig)` (refused by
  `spi_config_valid()`: CRC outside full duplex or with a zero
  polynomial, SSOE on a client), `enable()`, `disable()` - the drain
  procedure, TXE then not BSY, bounded -, `data()` both ways,
  `rxne()`/`txe()`/`busy()`, `flush_rx()`, the three errors with their
  clearing sequences (OVR: DATAR then STATR; MODF: STATR then a CTLR1
  write), the CRC verbs (`crc_next()`, `tx_crc()`, `rx_crc()`),
  `software_select()`, `high_speed_read()`, `dma_requests()`, the
  three interrupt enables and `isr()`, the raised-and-enabled sources.
- `SpiHost<1, pins, TxEngine, RxEngine>` is the engine `SpiBus` (=
  `BusMaster`) drives. Its `Request` carries the other strata's fields
  name for name: a chip select `PinRef` and a D/C line, a command phase
  (D/C low), a data phase with an optional out buffer (null = 0xFF
  dummies) and an optional in buffer (null = discard), the per-request
  `mode`, `clock` and `bits`, and `polled` - false runs the frames on
  the RXNE interrupt with the kernel free between them, true spins
  inside `start()` and completes synchronously. The ORDER of the fields
  is this host's: forty bytes with no padding, the eight words an
  asynchronous tenure needs first, then one word holding `clock`,
  `mode` and `bits` behind `cs_setup_us`, then the reply (the next
  section). `status()` answers `spi_ok`, `spi_dma_fault`,
  `spi_overrun` (the pump lost a frame to the one-deep receive buffer:
  every frame went out, one that came back is missing) or
  `spi_stalled` (a polled flag never came within the transaction's
  budget). `init(clock, max_sck_hz)`, `rebase()`, `clock_for(hz)`,
  `prime()` for a caller framing its own select, `bit_order()`,
  `isr()`, `dma_isr()`, `recover()`, `release()`, `claim_nss_pad()`.
  The engine slots are `DmaTxEngine<3, Elem>` and `DmaRxEngine<2,
  Elem>`, both or neither, on those two channels and no others (refused
  at compile time otherwise). Their `Elem` is the widest beat:
  `uint8_t` engines serve the 8-bit requests and send a 16-bit one to
  the pump; `uint16_t` engines - DATAR's width, nothing wider compiles
  - serve both, the 16-bit request in half-word beats when both of its
  buffers sit on a half-word boundary and on the pump otherwise. The
  transmit engine is armed for its errors alone: ONE DMA INTERRUPT A
  TRANSACTION, the receive channel's, and `dma_isr()` reads the
  controller's one flag register once for both channels. The data
  phase's launch is the two engines' block starts and the two DMA
  requests raised around them, no mask (nothing completes under it).
  Which requests the engines take at all is a count (`dma_min_frames`,
  `dma_min_frames_polled`, the next section). Frames in a byte buffer:
  one byte per 8-bit frame, two bytes low-first per 16-bit frame - one
  half-word in memory, which is what a 16-bit beat moves.
- `SpiClient<1, pins>`: the target side, thin - `init(clock, Config)`,
  `enable(first)` with the first answer loaded before the host's
  clock, `write()` on TXE (ONE frame ahead: no FIFO), `poll()`,
  `selected()` read off the NSS pad, `drive_output()` for a dark
  listener, `isr()`.

[brio/ch32v00x/pin.hpp](../../brio/ch32v00x/pin.hpp)'s `PinRef` (the
runtime pin the request carries) and `Pad` (the compile-time pad name
the pin tables use) were born with this driver.

## The host above the wire

What a transaction costs beyond its frames' time is the host's own:
`start()`, the polled loops, the pump's handler. Each is written from
16.2.2's two sentences above, each is counted in the release listings
(WCH gcc 15.2, `-Os`, the `xw` ISA, the hardware prologue on:
`test_ch32_spi.lst` and `bench_ch32.lst` for the CH32V006, the cost
model 2.5 cycles a straight-line instruction, the prologue's entry 29
and exit 25 cycles measured in platform.md), and each is MEASURED on
the CH32V006K8U6 at 48 MHz by `bench_ch32`'s letters d and e, MISO
floating. A bench line holds its instrument: the stopwatch's 44 cycles
in every wall, and on a pumped or engined line the meter's stamp pair,
113 cycles a handler, of which `enter()` - the handler's first
statement - lies on the bus's dead time (letter r).

- **The request is lent for the call** (design/spi-bus.md): a polled
  request completes inside `start()` and is read through the reference
  - copied nowhere; an asynchronous one has its tenure copied as eight
  word loads and eight word stores inline (the two pins, the three
  buffers, the two lengths, the completion style), never a call. The
  descriptor's layout makes those words its first eight and leaves no
  padding: 40 bytes where the same fields in the other strata's order
  take 48 with twelve of padding, and the copy a transaction paid - a
  48-byte `memcpy` call - is gone from every path.
- **`apply()` compares one word.** The request's `clock`, `mode` and
  `bits` lie together behind `cs_setup_us` at a word boundary, so the
  compare with the last request's is one load and a shift against a
  kept word: nine instructions with the ceiling's clamp, nothing
  written. On a mismatch the three are folded into CTLR1 and compared
  with the word in force (two requests under a ceiling may fold to the
  same word); a changed word is written with the drain and the
  disable/enable pair 16.3.1 asks for DFF.
- **Two polled loops.** A phase whose answers nobody wants - the
  command phase, a write with no in buffer - runs the TRANSMIT-ONLY
  loop: a frame written whenever TXE says the buffer is free, which it
  is a whole frame time before the shifter needs it, so the clock never
  pauses; the answers overrun the receive buffer, harmless to the
  transmit side by 16.2.7, and the tail waits TXE then not BSY and
  clears RXNE and OVR with the chapter's DATAR-then-STATR read. Nine
  instructions a frame at 8 bits (twelve at 16), and MEASURED AT THE
  WIRE at HCLK/4 and HCLK/16 in both widths: 31.9 cycles an 8-bit
  frame against the wire's 32 and 64.0 a 16-bit one against 64 at
  HCLK/4, 128 and 256 at HCLK/16 (HCLK/2 on 16-bit frames is the
  count's). An interrupt that lands in the loop only pauses the stream,
  and costs what it outlasts the frame's slack: the console's
  interrupts inside a 16-bit run at HCLK/4 cost 3 cycles a frame (67.1
  against 64.0). A phase with an in buffer runs the RECEIVE loop with
  ONE FRAME IN FLIGHT: write k, poll RXNE, read k and write k + 1 in
  the next instruction, the next frame's load and the store of the one
  read placed inside the wire time. Twelve instructions a frame at 8
  bits, of which four on the bus's dead time (the poll's last turn, the
  DATAR read, the DATAR write): about 15 cycles a frame plus the poll
  turn's phase, which the rate decides. Measured: 54 cycles a frame at
  HCLK/4, 1.69 times the wire's 32 (22 cycles of dead bus), and 144 at
  HCLK/16, 1.13 times (16 of dead bus); 87 and 285 a 16-bit frame, 1.36
  and 1.11. A frame AHEAD in
  that loop would be wire-bound but is refused by the same arithmetic
  as the pump's: two in flight lose a frame to any interrupt longer
  than a frame time, and a lost answer is a wrong read where an idle
  bus is only a slower one. Both loops spend ONE bounded budget of poll
  turns for the whole transaction (a frame's cycles per frame, a shift
  and no multiply) and end with `spi_stalled` when a flag never comes.
- **The select and D/C edges** are one store each into BSHR and BCR
  (RM 7.3.1.4, 7.3.1.5), inline, the null D/C a branch; `cs_setup_us`
  is tested before anything is called; no flush of the receive buffer
  at the start of a transaction, because every path leaves it empty at
  its end (the receive loop and the pump read every frame, the
  transmit-only loop flushes after its tail, the receive engine takes
  every frame, the fault exits flush) and `init()` and `recover()`
  flush what a reset or a `prime()` left.
- **The pump and its threshold.** One interrupt a frame (no FIFO); in
  the handler the DATAR read is the seventh instruction of the body
  and the next frame's write - PREPARED by the previous handler inside
  the wire time - the eighth after it; the store, the walk to the frame
  after and the counters come behind the write: 42 instructions on an
  8-bit frame with an in buffer, about 160 cycles a frame with the
  prologue and epilogue - measured 164 cycles of metered body a frame,
  one ruler read of 49 inside it. Above a rate threshold the handler
  keeps TWO frames in flight (the phase primed with two, the handler
  for frame k writing k + 2), so the bus never idles through the
  handler; below it one, the bus idle from RXNE to the write - about 70
  cycles, measured 253 cycles an 8-bit frame at HCLK/16 against the
  wire's 128, 125 of dead bus with the meter's `enter()` on it. The
  threshold is arithmetic in the header, its inputs named: two in
  flight is safe where a frame outlasts the longest window the SPI's
  service waits behind plus the core's entry plus the handler's path to
  the read. On this core NO INTERRUPT NESTS, so the first term is the
  longer of the image's longest handler and its longest masked window:
  the console transport's receive path with its RxActivity post, about
  245 cycles behind the prologue and epilogue; the kernel's post() of a
  48-byte bus request under the mask, about 160; the bench app's USART
  handler with its two stamps, about 300 - the constant is 320. The
  entry is the prologue's 29 and the nine instructions to the read, 52.
  A frame of more than 372 cycles: HCLK/64 and slower on 8-bit frames
  (512 cycles), HCLK/32 and slower on 16-bit ones. At HCLK/64 the pump
  is AT THE WIRE, measured: 512.0 cycles an 8-bit frame and 1024.0 a
  16-bit one, x 1.00 over 256 frames, `spi_ok` and no OVR on every
  run, with the console's and the tick's handlers live beside it. An
  OVR seen in the handler's STATR read ends the transaction with
  `spi_overrun`, the lost frame counted so the phase still ends - the
  witness of a threshold too low for the image it runs in, and the
  guard the constant has. THE PHASE IS PRIMED BEFORE THE INTERRUPT IS
  ARMED: with RXNEIE raised first, at HCLK/4 the first frame came back
  seventeen instructions before `begin_phase()` had stored the count
  left to write, the handler found nothing to write and the phase never
  ended (measured: one frame read, fifteen left, none in flight); armed
  after the prime, letter e pumps whole at every rate - 8.22 times the
  wire at HCLK/4 and 2.04 at HCLK/16, handler-bound (the 164-cycle
  handler against frames of 32 and 128), 1.00 at HCLK/64 with two in
  flight - `spi_ok` and no OVR on every line.
- **The engines against the pump**, two costs beside each other, both
  MEASURED: the engines' fixed cost a transaction, 518 cycles net of
  the instrument - an engined write of 16 or 256 frames at HCLK/4 or
  HCLK/16 runs 675 cycles above the wire's time with the request built
  before the stopwatch and the core spinning on the completion, less
  the stopwatch's 44 and the handler's stamp pair -, over the pump's
  164 a frame (the handler's metered body): `dma_min_frames` = 4 in the
  header, a data phase shorter than that taking the pump even with
  engines bound. A POLLED request takes the engines only with an in
  buffer and from `dma_min_frames_polled` = 23 frames on (the same fixed
  cost over the receive loop's 22 cycles of dead bus a frame); a polled
  write never does, its loop running at the wire. The count the header
  carried before the measurement, 355 cycles, took the launch's 59
  instructions and the completion vector and missed `start()`'s 44
  before the launch - the clamp, the one-word compare, the tenure's
  words, the frame step and the write-ahead decision - which brings it
  to about 465; the rest is the controller's and the shifter's latency
  at the block's two ends. Inside the block the engines are the wire's: 32.0
  cycles a byte at HCLK/4 and 128.0 at HCLK/16 in both widths, one
  interrupt a transaction. `bench_ch32`'s own letter d prints 878 above
  the wire: it builds the request inside the stopwatch (67 cycles, a
  40-byte `memset` among them) and idles through `BenchIdle` (136).
- **The fixed cost of a request**, a 3-byte polled one (a command and
  two data bytes, D/C scripted, the rate unchanged), MEASURED: 391
  cycles above the wire's 384 at HCLK/16 with the stopwatch's 44
  inside, about 350 the host's own; 299 for a lone command byte, 380
  for a command and 15 data bytes. The path in the listing is about 134
  instructions - `start()`'s 73 and the two phases' entries, frames and
  tails 61 - about 335 cycles at the model's rate, where the same
  request on the previous host ran about 250 instructions and a 48-byte
  `memcpy` call, about 650. The
  vendor's own loop, the EVT's `2Lines_FullDuplex` host side
  transcribed as a scratch program over the EVT's library (counted,
  not run): 51 instructions a frame through three library calls, about
  130 cycles - at the wire from HCLK/16, four times it at HCLK/4, where
  brio's transmit-only loop is at the wire and its receive loop at
  1.69.

## How to use it

```cpp
using Bus = brio::SpiHost<1>;                         // the default pads
using Spi = brio::SpiBus<Bus, P, 4>;                  // the arbiter AO
using Cs = brio::Pin<'C', 3>;

Bus::init(clock, 8'000'000);                          // a ceiling for the whole bus
Cs::output(true);

Bus::Request r{};
r.cs = Cs::ref();
r.cmd = brio::lend<brio::Lease::reply>(cmd);  r.cmd_len = 1;
r.tx = brio::lend<brio::Lease::reply>(pixels); r.len = 64;
r.clock = brio::SpiClock::div4;                       // 12 MHz at 48
r.mode = brio::SpiMode::mode0;
r.reply = brio::reply_to<Display, brio::SpiDone>();
brio::post<Spi>(r);

extern "C" BRIO_CH32_INTERRUPT void spi1_handler() {
    if (Bus::isr()) { brio::post<Spi>(brio::TransferDone{Bus::status()}); }
}
```

With the engines at DATAR's width, 8- and 16-bit frames both on them:
`SpiHost<1, spi1_default_pins, DmaTxEngine<3, uint16_t>,
DmaRxEngine<2, uint16_t>>`, and `dma1_channel2_handler` /
`dma1_channel3_handler` both calling `Bus::dma_isr()` the same way (the
receive channel's is the one that fires; the transmit channel's only on
a transfer error).

## Bench findings

The reference suite is `test_ch32_spi`, two instruments on the same
pads and never both: five letters on ONE JUMPER (MOSI PC6 to MISO
PC7, declining without it) and five against a PEER BOARD running
`spi_peer` on five wires (SCK PC5, MOSI PC6, MISO PC7, the GPIO chip
select PC3 to the peer's SS, GND - the other strata's spi_link
protocol, commanded in band), plus the wireless letter and a slip
probe outside `z`. On the CH32V006K8U6 at 48 MHz the image is one; on
the CH32V003F4P6 it is
seven group images (letters a, b and e; c and d; f; n and o; p; q; r
and x - the part's 15 KB and 2 KB decide the cut, design/overview.md,
and the kernel letter against the peer is alone because its stack
wants the room). Both instruments have been on both boards, with the
same numbers on each. The jumper is PROBED AT EVERY LETTER, never
trusted from the boot (the desk changes hands between letters: the
wires moved to the peer under a running image once, and a cached
"present" sent the loop letters against the peer); the probe pulls
MISO AGAINST the level it drives on MOSI, because a floating PC7
echoes PC6 with no wire (measured on the CH32V003F4P6). The peer
findings below are against a SAM C21 at 3.3 V, and a missing ground
wire between the boards shows as an intermittent link with the peer
nak-ing corrupted frames (measured), not as silence.

- **The reset values are table 16-2's** (STATR 0x0002 with TXE up,
  CRCR 0x0007), and a control word lands as spelled.
- **Seven of seven CTLR1 fields take a write under SPE** - DFF, CRCEN,
  CPOL, CPHA, LSBFIRST, BR and MSTR - so the enable protection the
  chapter describes does not exist in the silicon.
- **Every path completes with nothing on MISO**: the ISR pump (sixteen
  interrupts for sixteen frames) and the polled path, each transaction
  ending and releasing the select.
- **On the loop, the pump**: the four modes at 8 and 16 bits, sixteen
  frames each, byte-exact with one interrupt per frame; a two-frame
  command phase then eight data frames, the data exact and the
  command's echo discarded; a read with no out buffer clocks 0xFF
  dummies; the select released after every transaction.
- **The polled path at every rate**: all eight BR codes carry 64
  bytes byte-exact through the receive loop, timed with the request's
  fixed cost spread over the 64 frames: 44 cycles a frame at HCLK/2
  against the wire's 16, 64 at /4, 86 at /8, 169 at /16, 295 at /32,
  548 at /64.
- **The engines on the loop**: a 128-byte block through both at HCLK/2
  byte-exact with ONE DMA interrupt and no fault on either channel
  (2916 cycles, 868 above the wire's 2048); a command frame on the pump
  then 32 data frames on the engines, exact; a polled request on the
  engines completing inside `start()`; 32 frames of 16 bits in
  half-word beats with no SPI interrupt and one DMA interrupt; a 16-bit
  request off a half-word boundary on the pump, sixteen entries; a read
  with no out buffer clocking the fixed 0xFF cell at both widths.
- **The hardware CRC is the arithmetic**: TXCRCR over six frames is
  what a bitwise loop over the same polynomial computes (0x5A), the
  receiver's RXCRCR over the looped-back frames is the same number,
  the CRC frame read back is that value and CRCERR stands down.
- **The kernel over it, unchanged**: four transactions through SpiBus
  with four replies, ISR-pumped and polled interleaved on one bus; six
  posted into a four-deep queue, one rejected at once and every
  request answered exactly once; an idle bus votes for the sleep and
  a busy one against it.
- **The peer's command channel** at HCLK/256 = 187 kHz: the ident
  names spi_peer on another architecture, ten of ten pings answered,
  one frame per select window.
- **The matrix against the peer**: all four transfer modes byte-exact
  in both directions, LSb first with both ends agreeing byte-exact
  too, and a bit-order mismatch an EXACT two-way bit reversal; 0 of 40
  bursts slipped over ten rounds of each mode, on either part.
- **The BR ladder against the peer** holds to HCLK/4 = 12 MHz exact
  both ways and breaks at HCLK/2 = 24 MHz - where the peer still hears
  every character exact, so the boundary is its answer reload and not
  the wire.
- **The kernel against the peer**: four transactions queued from one
  dispatch in ONE select window come back in order, every one spi_ok,
  the 32 bytes read by the peer board byte-exact.
- **The roles invert**: this board as a CLIENT on the pads it hosts
  with, SOFTWARE-SELECTED (the NSS pad of the default column is PC1,
  the I2C's SDA on a desk carrying both buses), reads twelve frames
  the foreign host clocks at 1.5 MHz byte-exact, and the host reads
  the answer stream byte-exact from the first frame. THE SELECT EDGE
  IS THE START EVEN WITHOUT A SELECT PAD (measured): a client enabled
  during the peer's lead-in counted the peer's own pad reconfiguration
  as a clock edge and read every frame one bit late (0x16 for 0x2C),
  so the shifter is enabled only once the peer's SS reads low on PC3,
  inside the 20 us it leaves before the first byte.

## Not covered yet

Driver gaps, each with its reason:

- The simplex modes (BIDIMODE/BIDIOE, RXONLY) beyond the resource's
  configuration bits: no user.
- The hardware select arrangements as the ENGINE's select: the engine's
  select is a GPIO on purpose (docs/design/spi-bus.md); the resource
  has SSOE and the multi-host input for a program that wants them.
- The CRC as part of a Request: the resource's verbs exist, born into a
  tenure shape with a device that checks one.
- The client on its hardware NSS input against a foreign host: the
  default column's NSS pad is the I2C's SDA on the desk, so the peer
  letters select the client in software; the remap column (NSS on
  PC0) would free it, on a desk without the LED there.

Implemented but not bench-verified, each with what would measure it:

- The pump at HCLK/2 on a loop letter: `bench_ch32`'s letter e pumps
  at HCLK/4, /16 and /64 whole (the finding above: the interrupt is
  armed after the phase's prime, since a handler let in before the
  count's store found nothing to write and the phase never ended -
  measured at HCLK/4 before the order was fixed), and no
  `test_ch32_spi` letter pumps faster than HCLK/8; HCLK/2 at either
  width is inside the same window by the count. What would measure it:
  a loop letter pumping at HCLK/2 and HCLK/4, the frames judged.
- The host above the wire on the CH32V003F4P6 - `bench_ch32`'s letters
  d and e and `test_ch32_spi`'s loop letters on its group images - and
  the peer letters o, p and q on either part against `spi_peer`
  byte-exact through the hosts as they are: that board is off the
  desk, and the peer shares the pads with the jumper.
- `spi_overrun` and `spi_stalled` as exits: no run raised either (no
  OVR on any pumped line, no flag late on any polled one). Two frames
  in flight under a handler longer than a frame, and a polled request
  with the block held in reset under it, would provoke each.
- The vendor's loop on the same board, counted above: its run beside
  letter e.
- HSCR's high-speed read mode: its rate formula against a scope on
  SCK.
