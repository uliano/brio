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
- **The client is restricted, and not as 16.2.3 says.** "If you need to
  send in slave mode, you need to set CPOL to mode 2 or mode 3" - and
  measured against a foreign host (the CH32X035's, below), this client is
  served in modes 0 and 3 BOTH WAYS, MSb and LSb first, and in modes 1 and
  2 NEITHER way: what it hears there is every byte one bit early (0x14
  read 0x28), as a client that samples on the rising edge in every mode
  would, and its MISO a fixed pattern whatever it was given. The host's
  own waveforms are the standard's in all four modes (measured by the far
  chip). `SpiClient` refuses modes 1 and 2 on the CH32V006.

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
- `SpiHost<1, pins, TxEngine, RxEngine, hold_off_cycles, Placement>` is the
  engine `SpiBus` (= `BusMaster`) drives. Its `Request` carries the other strata's fields
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
  `write_ahead_from(bits)` (the fastest code a receive of that width
  keeps two frames in flight at, the next section), `prime()` for a
  caller framing its own select, `bit_order()`, `isr()`, `dma_isr()`,
  `recover()`, `release()`, `claim_nss_pad()`. `hold_off_cycles` is
  the IMAGE's: the longest, in core cycles at the clock the host runs
  at, that the image keeps the host's vector from running - the longer
  of its longest handler and its longest masked window - with a default
  of 320 and a zero refused at compile time (the next section).
  `Placement` is where the two polled loops run from,
  util/placement.hpp's `InFlash` (the default) or `InRam` - each width
  of each loop then one instance in SRAM, a call a phase
  ([platform.md](platform.md), "A handler in SRAM"); any other tag is
  refused at compile time. The pump is not placed: it is inline in the
  vector under either tag (the next section).
  The engine slots are `DmaTxEngine<3, Elem>` and `DmaRxEngine<2,
  Elem>`, both or neither, on those two channels and no others (refused
  at compile time otherwise). Their `Elem` is the widest beat:
  `uint8_t` engines serve the 8-bit requests and send a 16-bit one to
  the pump; `uint16_t` engines - DATAR's width, nothing wider compiles
  - serve both, the 16-bit request in half-word beats when both of its
  buffers sit on a half-word boundary and on the pump otherwise. The
  transmit engine is armed for its errors alone: ONE DMA INTERRUPT A
  TRANSACTION, the receive channel's, and `dma_isr()` reads the
  controller's one flag register once for both channels. The receive
  engine arms at very_high and the transmit at high
  ([../design/dma.md](../design/dma.md)): DATAR's receive half holds one
  frame, and a receive channel the transmit one outran would find the
  next frame on top of its last (OVR, 16.2.7), while a transmit channel
  that waits only stops the host's clock between two frames. The data
  phase's launch is the two engines' block starts and the two DMA
  requests raised around them, no mask (nothing completes under it).
  Which requests the engines take at all is a count (`dma_min_frames`,
  `dma_min_frames_polled`, the next section). Frames in a byte buffer:
  one byte per 8-bit frame, two bytes low-first per 16-bit frame - one
  half-word in memory, which is what a 16-bit beat moves.
- `SpiClient<1, pins>`: the target side - `init(clock, Config)`
  refusing a mode this part's client is not served in (`config_served()`,
  `spi_client_mode_served()`: modes 1 and 2 on the CH32V006) and
  `init<cfg>(clock)`, where such a mode does not compile; the per-frame
  verbs - `enable(first)` with the first answer loaded before the host's
  clock, `enable()` with the buffer empty, `write()` on TXE (ONE frame
  ahead: no FIFO), `poll()`, `selected()` read off the NSS pad,
  `drive_output()` for a dark listener, `isr()` -; and THE RUNS a
  transaction is served by, its unit (the next section but one):
  `answer(out, spins)`, `exchange(out, in, spins)`, `receive(in, spins)`
  over `uint8_t` or `uint16_t` frames, each returning a `Run` - the
  frames moved and how it ended, `complete`, `closed` (the host raised
  the select) or `stalled` (the spin budget). A run over a wider element,
  and an answering run on a client whose pins name no MISO pad, do not
  compile (`test/family_ch32v00x/neg/spi_client_*.cpp`).

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
the CH32V006K8U6 at 48 MHz by `bench_ch32`'s letters s and e, MISO
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
  turn's phase, which the rate decides. Measured: 56.7 cycles a frame at
  HCLK/4, 1.77 times the wire's 32 (25 cycles of dead bus), and 149 at
  HCLK/16, 1.16 times (21 of dead bus); 91 and 286 a 16-bit frame, 1.42
  and 1.12. A frame AHEAD in
  that loop would be wire-bound but is refused by the same arithmetic
  as the pump's: two in flight lose a frame to any interrupt longer
  than a frame time, and a lost answer is a wrong read where an idle
  bus is only a slower one. Both loops spend ONE bounded budget of poll
  turns for the whole transaction (a frame's cycles per frame, a shift
  and no multiply) and end with `spi_stalled` when a flag never comes.
- **The loops from SRAM** (`Placement` = `InRam`, measured in the same
  letter through a twin host): the transmit-only loop at the wire from
  HCLK/2, 16.0 cycles an 8-bit frame against the flash's 24.9 (the
  wire's 16) and 33.7 a 16-bit one against 37.9 (32); the receive loop
  42.8 cycles an 8-bit frame at HCLK/4 against 56.8 - 11 cycles of dead
  bus a frame where the flash leaves 25 - 148.4 against 166.7 at
  HCLK/16, 83.5 against 104.1 a 16-bit frame at HCLK/4, and 32.9
  against 34.9 at HCLK/2, where the poll's turn is the wire's frame.
  446 bytes of SRAM a host type, both loops at both widths, each called
  once a phase. The polled threshold follows the placement:
  `dma_min_frames_polled` 20 from the flash, 45 from SRAM (the same
  engines' fixed cost over 11 cycles a frame).
- **The pump is not placed by the host.** Inline in the vector, 167
  cycles of metered body a frame; the same body as an instance in SRAM
  behind the same vector in the flash took 176 (274.4 cycles a frame
  at HCLK/4 against 259.4): the call into SRAM and the return into the
  flash behind its wait states cost the short body more than SRAM saves
  it. A program that wants it from SRAM places the vector
  ([platform.md](platform.md), "A handler in SRAM"): 174.7 cycles a
  frame at HCLK/4, 5.5 times the wire against 8.1.
- **The select and D/C edges** are one store each into BSHR and BCR
  (RM 7.3.1.4, 7.3.1.5), inline, the null D/C a branch; `cs_setup_us`
  is tested before anything is called; no flush of the receive buffer
  at the start of a transaction, because every path leaves it empty at
  its end (the receive loop and the pump read every frame, the
  transmit-only loop flushes after its tail, the receive engine takes
  every frame, the fault exits flush) and `init()` and `recover()`
  flush what a reset or a `prime()` left.
- **The pump and its threshold.** One interrupt a frame (no FIFO); in
  the handler the DATAR read is the fifth instruction of the body, the
  STATR load that tests OVR the next - a status read AFTER the data
  read, so that a frame completing at any moment before the read is
  seen and 16.2.7's clear (DATAR then STATR) runs in the same handler -
  and the next frame's write - PREPARED by the previous handler inside
  the wire time - the ninth after the read; the store, the walk to the
  frame after and the counters come behind the write: 42 instructions on an
  8-bit frame with an in buffer, about 160 cycles a frame with the
  prologue and epilogue - measured 177 cycles of metered body a frame,
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
  handler with its two stamps, about 300. That first term is a fact of
  the IMAGE and not of the family, so it is the host's template
  parameter `hold_off_cycles`, declared by the application or its board
  file, and 320 - the bench app's handler, rounded up - is only its
  default. The entry is the prologue's 29 and the seven instructions to
  the read, 48. Under the default, a frame of more than 368 cycles:
  HCLK/64 and slower on 8-bit frames (512 cycles), HCLK/32 and slower
  on 16-bit ones; an image that declares 600 gets /128 and /64 (both
  pinned at compile time). An image measures its own term with
  util/bench.hpp's `IsrMeter` in each of its vectors - `bench_ch32`
  meters the tick (letter t), the console (p), the DMA's vectors (d,
  s) and the pump (e), `isr` over `irq` a handler's body, the
  hardware prologue's entry and exit ([platform.md](platform.md)) on
  top - and its longest masked window in its listing. A hold-off of
  zero does not compile: an image with no handler of its own still has
  the tick's. At HCLK/64 the pump
  is AT THE WIRE, measured: 512.0 cycles an 8-bit frame and 1024.0 a
  16-bit one, x 1.00 over 256 frames, `spi_ok` and no OVR on every
  run, with the console's and the tick's handlers live beside it. An
  OVR seen in the STATR read after the DATAR read gives the transaction
  `spi_overrun`, the lost frame counted so the phase still ends - the
  witness of a hold-off declared too short for the image it runs in.
  Measured (`bench_ch32`'s letter e, `spi.held`, MISO floating): a
  pumped receive of 1024 8-bit frames at HCLK/32 - 256 cycles a frame,
  between the two thresholds - eight runs with a console line in
  flight, on the default host (one frame in flight there) `spi_ok` in
  8 of 8, on a host declaring 100 (two in flight from HCLK/32)
  `spi_overrun` in 8 of 8: the console's handler, about 300 cycles
  with its stamps, outlasts the frame the second one in flight leaves
  it, while the pump's own (177 cycles a frame) does not. THE PHASE IS PRIMED BEFORE THE INTERRUPT IS
  ARMED: with RXNEIE raised first, at HCLK/4 the first frame came back
  seventeen instructions before `begin_phase()` had stored the count
  left to write, the handler found nothing to write and the phase never
  ended (measured: one frame read, fifteen left, none in flight); armed
  after the prime, letter e pumps whole at every rate - 8.27 times the
  wire at HCLK/4 and 2.06 at HCLK/16, handler-bound (the 177-cycle
  handler against frames of 32 and 128), 1.00 at HCLK/64 with two in
  flight - `spi_ok` and no OVR on every line.
- **The engines against the pump**, two costs beside each other, both
  MEASURED in one session (letters s and e, three runs each): the
  engines' fixed cost a transaction, 504 cycles net of the instrument -
  an engined write of 16 or 256 frames at HCLK/4 or HCLK/16 runs 864
  cycles above the wire's time on letter s, which carries 360 of
  instrument (the request built inside the stopwatch 67, `BenchIdle`
  136, the stopwatch's 44, the handler's stamp pair 113) -, over the
  pump's 177 a frame (the handler's metered body): `dma_min_frames` = 3
  in the header, a data phase shorter than that taking the pump even
  with engines bound. A POLLED request takes the engines only with an
  in buffer and from `dma_min_frames_polled` = 20 frames on (the same
  fixed cost over the receive loop's 25 cycles of dead bus a frame); a
  polled
  write never does, its loop running at the wire. The count the header
  carried before the measurement, 355 cycles, took the launch's 59
  instructions and the completion vector and missed `start()`'s 44
  before the launch - the clamp, the one-word compare, the tenure's
  words, the frame step and the write-ahead decision - which brings it
  to about 465; the rest is the controller's and the shifter's latency
  at the block's two ends. Inside the block the engines are the wire's: 32.0
  cycles a byte at HCLK/4 and 128.0 at HCLK/16 in both widths, one
  interrupt a transaction. `bench_ch32`'s own letter s prints 864 above
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

## The client's runs

A client's unit is the run: the answers to a transaction, or what it
brings, served polled in one call. What each costs is counted in the
release listing of `spi_peer`, which inlines all three at both widths
(WCH gcc 15.2, `-Os`, the `xw` ISA), and measured from the far side by
`test_x035_spi`'s letter k: the CH32X035 clocks 32 frames of 8 or 16 bits
under its polled loop - a gap after every frame, the control - and back
to back under its engines, every rung from /64 to /2, and judges each end.

| offer (section) | taken? | why |
|---|---|---|
| TXE, the buffer moved into the shifter at a frame's first sampling edge (16.2.3) | yes | answer k + 1 is written on frame k's TXE: one frame of window, one answer ahead (no FIFO, figure 16-1) |
| RXNE and the one-deep receive buffer (16.2.3, 16.2.7) | yes, read in the middle of a frame | the exchange reads frame k - 1 on its standing RXNE right after frame k's answer is written: a DATAR read at a frame's end lost the next answer, and TXE alone does not say RXNE (both measured, below) |
| the NSS pad (16.2.1) | yes, the window | the chapter has no flag for a client's window end: the pad is read once for every four polls of a flag |
| DMA requests on channels 2 and 3 (table 8-2) | not in a run | a DMA serve is the program's composition of the engines over the resource - its binding, its channels and the latched-request rule (What the silicon does) are the program's - and `spi_peer`'s default serve, at the wire to 24 MHz; a run is the polled serve, with nothing to arm |
| RXNE and TXE interrupts (16.2.8) | not in a run | an interrupt a frame pays the hardware prologue and epilogue, 54 cycles, before a body, against frames of 64 cycles at 6 MHz; `isr()` stands for a program that wants them |
| SRAM execution | the program's | the runs are always inline, so a program places the function that calls them: a plain function with `[[gnu::section(".fast_text")]]`, an inline or template one through `Hot<InRam>::run` ([platform.md](platform.md), "A handler in SRAM"); `spi_peer` keeps its caller in the flash, behind two wait states at 48 MHz (RM 18.3.1), where the fetch is a run's ceiling (measured, below) |

THE COST, in the steady state with each flag found at its first poll,
no call anywhere in a run:

- the EXCHANGE, 13 instructions a frame: the next answer's load, the TXE
  poll's load, test and branch, the DATAR store, the RXNE test's load,
  test and branch (one poll: the flag stands by then), the DATAR
  load, two pointer steps, the frame's store and the loop's branch -
  three taken branches, four peripheral accesses (STATR twice, DATAR
  twice) and two SRAM ones;
- the ANSWER run, 7: the load, the poll's three, the store, the step,
  the branch - two taken, two peripheral accesses;
- the RECEIVE run, 7: the poll's three, the DATAR load, the step, the
  store, the branch - two taken, two peripheral accesses.

A poll that misses is three instructions more, and every fourth a look
at the pad (one load of the port's input register) and the budget's
step. The answer is loaded before the TXE poll, so the store follows the
load that sees the flag with nothing between them.

THE CEILINGS, every rung up to the one named exact and the next one not,
in three runs of the letter on the final images (the tick held, below):

| run | 8-bit, gapped | 8-bit, back to back | 16-bit, gapped | 16-bit, back to back |
|---|---|---|---|---|
| exchange | 6 MHz | 6 MHz | 12 MHz | 12 MHz |
| answer | 24 MHz | 12 MHz | 24 MHz | 24 MHz |
| receive | 24 MHz | 12 MHz | 24 MHz | 24 MHz |

In cycles: the exchange holds a frame of 64 cycles and not one of 32,
the answer and receive runs one of 32 and not one of 16 (the gapped
host's gap after each frame carrying them to 24 MHz in 8-bit frames).
Thirteen instructions in 33 to 64 cycles is 2.5 to 5 cycles an
instruction, the three taken branches refetching from the flash behind
its wait states. The receive run is at the edge of its window there: its
12 MHz back-to-back rung lost one frame in two runs of nine across the
session and its 24 MHz gapped one in one. The wire is not what stops
them: from SRAM the same receive run hears 24 MHz back to back (below).

- **A DATAR READ AT A FRAME'S END LOSES THE NEXT ANSWER.** With frame k
  read on its own RXNE - at the frame's end, a bit before answer k + 1
  moves into the shifter - back to back at 12 MHz in 16-bit frames the
  far host read 56 to 58 of 64 bytes wrong in five runs of five, the
  answer before repeated, while this client heard all 32 frames; with
  frame k - 1 read after the TXE of frame k, the same rung exact in
  every run with the tick held (six). The mechanism inside the block is
  not isolated (a logic analyser on SCK and MISO would show which edge
  the transfer takes).
- **TXE DOES NOT SAY RXNE.** Reading frame k - 1 on the TXE of frame k
  with no RXNE test - one poll a frame - under the gapped host at 12 MHz
  in 8-bit frames, 9 and 10 of the 32 frames read were wrong in two runs
  while every answer was right: the exchange tests RXNE before its read,
  one poll when the flag stands, as it does by then.
- **THE TICK COSTS A FRAME back to back at 6 MHz in 8-bit frames**: its
  handler, a dozen instructions behind the hardware prologue and epilogue
  (about 85 cycles), outlasts a frame of 64. With the tick live two
  letter runs of eight lost one frame at that rung, in the exchange once
  and in a receive run once; `spi_peer` holds its tick for a polled serve,
  and no run since lost one. A program's runs back to back are exact
  while no handler of its image outlasts a frame - the image's hold-off,
  as for the host's pump.
- **PLACEMENT**: the same runs from SRAM (a measurement image with the
  peer's serve in `.data`, about 1.1 KB a frame width for the three runs
  inlined) - the exchange exact to 12 MHz in 8-bit frames under both
  hosts, the receive run to 24 MHz back to back, every 16-bit run to 24
  MHz: one rung, a factor of two, over the flash for the exchange. A
  program takes it by placing the function that calls the runs - a
  plain one with `[[gnu::section(".fast_text")]]`, an inline or template
  one through `Hot<InRam>::run` ([platform.md](platform.md), "A handler
  in SRAM"); the runs themselves stay inline in it.

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

A client answering a transaction in one call, on PC1's hardware select:

```cpp
using C = brio::SpiClient<1>;
C::init<C::Config{.mode = brio::SpiMode::mode3}>(clock);
const auto run = C::exchange(std::span<const uint8_t>(answers), std::span<uint8_t>(heard), spins);
if (run.end != C::RunEnd::complete) { /* run.count frames came */ }
```

## Bench findings

The reference suite is `test_ch32_spi` on the CH32V006K8U6 at 48 MHz, its
SPI1 column 0 (PC5 SCK, PC6 MOSI, PC7 MISO, PC1 the select) wired straight
to the CH32X035F8U6's SPI1 (PA5, PA7, PA6, PA4), the far chip running its
`spi_peer` ([../ch32x035/spi.md](../ch32x035/spi.md)) commanded in band
through the other strata's spi_link protocol - every frame on the wire
witnessed by a second chip, the 16-bit frames, the CRC and a receive-only
client being the protocol's extended exchange. `z` is 63 verdicts, green
whole twice on the final images: a, c, f and g with the peer deselected
(their select is PC3, a pad with no wire, PC1 held high), the other
letters against it; s (the pair's ceiling) and x (the slips: 0 of 40) run
by name. On the CH32V003F4P6 the suite is fifteen group images, one letter
each - the part's 15 KB with no multiplier decide the cut
(design/overview.md) - linked by `brio fit` and not run: that part is off
the desk.

- **The reset values are table 16-2's** (STATR 0x0002 with TXE up,
  CRCR 0x0007), and a control word lands as spelled.
- **Seven of seven CTLR1 fields take a write under SPE** - DFF, CRCEN,
  CPOL, CPHA, LSBFIRST, BR and MSTR - so the enable protection the
  chapter describes does not exist in the silicon.
- **Every path completes with nothing driving MISO**: the ISR pump
  (sixteen interrupts for sixteen frames), the polled path and the
  engines, each transaction ending and releasing its select.
- **The pump against a foreign client** (letter b): the four modes at 8
  and 16 bits, sixteen frames each, exact both ways at HCLK/16 with one
  interrupt a frame; a two-frame command phase then eight data frames -
  the client heard all ten, the data read back exact, the command's echo
  discarded; a read with no out buffer clocks 0xFF dummies, heard as such.
- **The polled path at every rate** (letter c, MISO undriven): all eight
  BR codes carry 64 frames to `spi_ok`, timed with the request's fixed
  cost spread over them: 50 cycles a frame at HCLK/2 against the wire's
  16, 65 at /4, 95 at /8, 155 at /16, 290 at /32, 545 at /64 - none
  faster than its wire, the receive loop's one frame in flight the rest.
- **The engines against the peer** (letter d, HCLK/8): a 128-frame block
  through both engines exact both ways with ONE DMA interrupt and no SPI
  one, no fault on either channel; a command frame on the pump then 32
  data frames on the engines; a polled request on the engines completing
  inside `start()`; 32 frames of 16 bits in half-word beats, the foreign
  client hearing 16-bit frames; a 16-bit request off a half-word boundary
  on the pump, sixteen entries; a read with no out buffer clocking the
  0xFF cell at both widths, heard as 0xFF.
- **A POLLED REQUEST ON THE ENGINES MOVES ITS OWN FRAME WIDTH**: the beat
  `launch_dma()` reads is the frame step, which the asynchronous path set
  and the polled one did not - measured: on an image without the polled
  path's step, a polled 16-bit block after an asynchronous 8-bit one and a
  polled 8-bit block after an asynchronous 16-bit one both came back wrong
  at both ends, every frame; with it, both exact (letter d).
- **THE RATE ON THE WIRE** (letter g, the peer deselected: ONE FRAME inside
  an engined block, the difference of two block lengths, best of three,
  against its SCK periods at HCLK over the divisor the BR code names,
  16.3.1, with no gap added - a frame written while another shifts
  follows it with none, 16.2.2): from SRAM buffers 1002, 998, 999 and
  1000 thousandths at HCLK/2, /4, /16 and /64 in mode 0, 1000 at /16 in
  mode 3, 998 and 1000 at /4 and /16 in 16-bit frames; and with no buffer
  named, on the host's own cells, 1000 at /2 and /4 and 999 at 16-bit /4.
  THE CELLS ARE IN SRAM: with the 0xFF cell a constant in flash the same
  block took 1031 thousandths at HCLK/2 - 16.5 cycles where 16 are the
  wire's, the transmit channel reading its source behind two wait states
  (RM V1.5 18.3.1) - and a transmit source in SRAM ran at 1000.
- **The hardware CRC against a foreign CRC unit** (letter e, HCLK/64):
  CRC-8 (0x07) over eight frames and CRC-16 (0x8005) over eight 16-bit
  frames, each end's TXCRCR the bitwise CRC of what it sent (0x62,
  0x5FB6), RXCRCR that of what came (0x5B, 0x2B13), the CRC frame this end
  read the far chip's bitwise CRC of its stream, CRCERR down at both ends;
  and on two polynomials (0x07 here, 0x31 there) CRCERR up at BOTH.
- **The kernel over it, unchanged** (letter f): four transactions through
  SpiBus with four replies, ISR-pumped and polled interleaved on one bus;
  six posted into a four-deep queue, one rejected at once and every
  request answered exactly once; an idle bus votes for the sleep and a
  busy one against it. Against the peer (letter q): four transactions
  queued from one dispatch in one select window, every byte read back by
  the far chip.
- **The matrix** (letter o, HCLK/256): the four modes MSb first exact both
  ways on both of the client's serves (its DMA engines and its polled
  run); this host LSb first against the MSb-first client an exact two-way
  bit reversal in every mode; LSb first at both ends heard exact by the
  client listening (it cannot answer so, and refuses to).
- **The ladder** (letter p, /256 to /2): the polled host and the engines
  exact both ways against the client's DMA serve to /8 = 6 MHz; this
  host's transmit-only paths heard whole by a listening client to /4 =
  12 MHz; the client's polled run to /16 behind the polled host and to
  /32 behind the engines' frames back to back (the /16 rung failed in
  one run of four) - the far chip's run reloads its answers from code in
  its flash. Above those rungs the pair is past its ceiling (below).
- **THIS CHIP AS CLIENT ON ITS HARDWARE NSS** (letter r: PC1 the NSS input,
  SSM clear, under the peer's `host_burst` at 1.5 MHz, one frame a
  request): in modes 0 and 3, MSb and LSb first, twelve frames received
  exact and the far host reading the answers exact from the first frame -
  the preload before the select fell; eight 16-bit frames in mode 3 the
  same; modes 1 and 2 refused by `SpiClient` before anything is commanded.
- **MODF from a second chip** (letter m): the peer's PA4 held low 2 ms
  demotes a host whose NSS is a hardware input 24 ms after the enable,
  MSTR and SPE cleared, four of four; STATR 0x22 before 16.2.7's clearing
  sequence and 0x02 after it, once the pad has been high a millisecond.
- **THE PAIR'S CEILING, the far client's MISO pad** (letter s, sixteen
  frames on this host's engines against the client's DMA serve): to /8
  every pattern is exact both ways. At /4 and /2 the client loses frames
  whenever its own MISO pad stands high: with its answers 0xFF it heard 0
  of 16 (its STATR showing the preload never taken), with 0x00 it heard
  16 of 16 at /4 AND /2; listening with its MISO pad undriven it heard
  16 at /4 and 6 at /2 - and with that pad held HIGH AS A GPIO (no SPI
  output at all) 0 at /4, held LOW 16 at /4 and /2. The four wires are
  not joined (each driven both ways against the others' pulls) and the
  MISO wire carries no load; the cause - the far chip's pad, the two
  boards' two rails, the wiring - is not isolated (the gap list).

- **The host above the wire on the CH32V003F4P6** (`bench_ch32` letters
  s and e, MISO floating, the loop idling through the platform's
  `idle()`): the engined write at the wire - 31.3 cycles a byte at
  HCLK/4 against 32, 126.5 at HCLK/16 against 128, one DMA interrupt,
  a fixed cost of 1160 to 1760 cycles a transaction - once the V2A's
  `idle()` stopped sleeping over a working channel (dma.md: before it
  the receive channel overran and the transaction never ended); the
  polled write 1.04 of the wire at 256 frames of HCLK/4; the pump
  with one frame in flight 576 cycles a frame at HCLK/4 and 579 at
  HCLK/16; and `spi.held` at 512 frames (the part's half buffer): the
  default host none not ok in eight runs, the host declaring 100
  eight. BUT THE PUMP WITH TWO FRAMES IN FLIGHT OVERRUNS UNDER THE
  V2A's SLEEP: at HCLK/64, where the default keeps two in flight, every
  run ended `spi_overrun` with the loop idling between frames - a
  Sleep of that core lasts at least 3150 cycles (platform.md), a hold-
  off ten times the default's 320 for an image that idles while a
  pump is in flight. `spi.held`'s runs wait in a spin and never sleep,
  which is why its default host is clean.
- **THE PUMP READS OVR AFTER THE DATA, and the window that reading it
  before left is closed** (`bench_ch32`'s letter e, `spi.window`, the
  CH32V006K8U6): a pumped write of 2 and of 3 8-bit
  frames at HCLK/32 and HCLK/64 by the host declaring 100 - two in
  flight - with SPI1's line masked at the PFIC around `start()` and
  unmasked after 0 to 511 empty turns, twice: the handler for frame 0
  enters before frame 1 completes, after it, and on it. With OVR tested
  in a status copy taken BEFORE the DATAR read, a completion between
  the two reads went unseen: 3 and 2 runs of 1024 never completed with
  two frames at HCLK/32 and HCLK/64 (the last frame lost uncounted, no
  interrupt to follow) and 1 with three at HCLK/32 (inside the run),
  and every two-frame run that
  lost its last frame left OVR standing for the next transaction (972
  of 1024 at HCLK/32, 889 at HCLK/64) - the clear's STATR half was the
  NEXT handler's, and the last frame has none. With the STATR load
  after the DATAR read: 1024 of 1024 completed on every line, 973 and
  892 of them with the last frame lost and counted, no OVR standing
  after any; a scratch probe in the handler counted the completions
  that fell between the two reads - 2, 2, 2 and 1 a line, every one
  completed. The load costs 6 cycles a frame: the handler's metered
  body 177 cycles at HCLK/16 against 171 with OVR tested from the first
  read (the same session), the pump 264.7 cycles a frame against 258.7
  at HCLK/4 and against 258.9 at HCLK/16; at HCLK/64 the pump is still
  at the wire.

### This part as the desk's instrument: `spi_peer`

`spi_peer` makes the CH32V006K8U6 the desk's SPI client for the
CH32X035's suite (`test_x035_spi`, its document
[../ch32x035/spi.md](../ch32x035/spi.md)): spi_link's command channel in
band over SPI1's column 0 with the HARDWARE select on PC1 (pulled up),
the dark listener, the exchange served on the DMA engines (channels 2 and
3, polled through the receive count, no interrupt) or on the client's
runs (its tick held for the run), `sink_slow`, `ss_pulse` on PC1 as a GPIO,
`host_burst` with PC1 the host's GPIO select; `mspi` refused (no USART of
this part reaches the wires). Modes 1 and 2, which `SpiClient` refuses on
this part, it serves all the same - the block configured by hand, the
resource's verbs on the same pads - so a far suite measures them; of the
extended exchange it serves the 16-bit frames (the engines in half-word
beats, the runs over `uint16_t`) and the receive-only run (MISO dark) and
refuses the CRC (`report_cfg_failed`, no host on the desk asking it),
raising `report_extended` on every exchange. This die's 24 MHz
crystal runs it (the PLL at 48 MHz), and its supply is read from VREFINT
at boot (3275 mV on the desk). What the far chip measured of this client,
on the four wires:

- **The hardware select frames it**: a client on PC1's NSS input took
  every exchange the far host clocked in modes 0 and 3, both bit orders,
  on both serves - eight frames exact both ways each time - and the far
  host's MODF letter took its select low four times on command.
- **Modes 1 and 2 are served neither way**: eight runs in each, every one
  wrong both ways - the first byte heard 0x28 for 0x14 (mode 1) and 0x2A
  for 0x15 (mode 2), one bit early, and the far chip's sampling of MISO
  in those modes a fixed 0x6D 0xB6 0xDB whatever the stream; the far
  host's MOSI the standard's in both (its waveform letter).
- **THE ANSWER WINDOWS ARE MODE 3.** In mode 0, an answer window served
  by the polled run right after a command came back exact, and the one
  served right after another answer window - ident's and report's data -
  came back one bit early, every byte (t << 1) | the next byte's MSB, the
  first bit racing; a DMA-served answer in mode 0 was exact. In mode 3
  every window was exact, so the peer serves ack, nak, ident and report in
  mode 3 - spi_link's alternate answer mode - and the far host finds that
  at its rendezvous; what in this
  block's state carries from one mode-0 window to the next is not
  isolated (a logic analyser on SCK and MISO across two windows would).
- **The rate ladder**: the DMA serve exact at every code to the far
  host's /2 = 24 MHz, behind its polled loop and its engines; the polled
  serve - the client's exchange run - exact to 6 MHz behind both, the
  run's own ceiling from the flash (The client's runs, above).
- **As a host** (`host_burst`, one polled frame a request, 20 us apart):
  its SCK and MOSI the standard's in all four modes and its frames exact
  at the far client, MSb and LSb first.

## Not covered yet

Driver gaps, each with its reason:

- The simplex modes (BIDIMODE/BIDIOE, RXONLY) beyond the resource's
  configuration bits: no user.
- The hardware select arrangements as the ENGINE's select: the engine's
  select is a GPIO on purpose (docs/design/spi-bus.md); the resource
  has SSOE and the multi-host input for a program that wants them.
- The CRC as part of a Request: the resource's verbs exist, born into a
  tenure shape with a device that checks one.
- The CH32V003F4P6's client in modes 1 and 2: `SpiClient` refuses them on
  the CH32V006 alone, where they were measured served neither way; the
  CH32V003 shares the block and has not answered on its own die, and a
  refusal stated for a part nobody measured would be a guess.

Implemented but not bench-verified, each with what would measure it:

- The client's runs on the CH32V003F4P6: they compile there (`brio check
  ch32v00x`); a CH32V003 wired to the CH32X035 as the CH32V006 is, under
  letter k, would measure them.
- A run on a client with no NSS pad (it ends on its count or its budget
  alone): it compiles; a client whose pins name no select, under a host
  framing its own, would run it.
- A run's caller placed in `.fast_text`: the rung it buys was measured
  with the caller moved into `.data` by hand (the exchange to 12 MHz in
  8-bit frames, about 1.1 KB of SRAM a frame width); `spi_peer` built
  with its `polled_serve()` in `.fast_text`, under `test_x035_spi`'s
  letter k, would measure the spelling.

- The suite on the CH32V003F4P6: its fifteen group images link and fit
  (`brio fit`); the part is off the desk, and a CH32V003 wired to the
  CH32X035 as the CH32V006 is would run them.
- The pump's OVR test after the DATAR read on the CH32V003F4P6: the
  code is the CH32V006's, and `spi.window`, which measures it there, is
  not in the 16 KB part's letter-e image (no room); a scratch image
  carrying that line alone on the CH32V003 would measure it.
- The data both ways above 6 MHz: the pair's ceiling is the far client's
  (bench findings, letter s), so the pump at HCLK/4 and /2 and the
  engines' exchanges at /4 and /2 are judged by no witness - letter g
  times the engines' frames at /2 on the wire alone. A client served to
  24 MHz - the CH32X035 with its MISO pad's effect explained (a scope on
  its PA5 with PA6 high at 12 MHz, the two boards on one rail), or
  another part - would judge them.
- The CH32V003's default hold-off against its Sleep: the pump's two
  frames in flight overrun when the image idles between frames (the
  finding above), and nothing in the host or the platform keeps them
  apart yet - a default that counts the Sleep's 3150 cycles, or an
  `idle()` that does not sleep while a pumped transaction is in
  flight, is the choice; `bench_ch32` letter e at HCLK/64 measures
  either.
- `spi_stalled` as an exit: no run raised it (no flag late on any
  polled line). A polled request with the block held in reset under it
  would provoke it. (`spi_overrun` is measured: the hold-off declared
  too short, above.)
- The vendor's loop on the same board, counted above: its run beside
  letter e.
- HSCR's high-speed read mode: its rate formula against a scope on
  SCK.
