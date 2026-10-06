# The benchmark

A driver's cost is part of what it is ([overview.md](overview.md), "A
driver's cost is part of what it is"): cycles per byte or per frame,
interrupts per byte, cycles under the mask, stated beside the wire's own
figure. This page is the instrument those numbers are read with - one
grammar on every family, so that a number read on one chip means what
the same number means on another, and the vendor's library doing the
same operation on the same board prints a line the same eyes read.

Contract and helpers: `util/bench.hpp` (pure, host-tested in
`test/test_bench`). One app per build project, `bench_<family>`, carries
the operations; each family's document states which of its drivers the
app measures and what the numbers were.

## The line

    bench <op> n=<bytes> wall=<cycles> busy=<cycles> irq=<count> isr=<cycles> rate=<B/s> wire=<B/s> x=<wall/wire>

- `wall` - ruler cycles from the operation's start to its completion,
  the idle included.
- `busy` - the cycles the core was not idle (the formula below).
- `irq` - the interrupts the operation took, counted in the bound
  vectors.
- `isr` - the cycles spent inside those handlers: a part of `busy`,
  told apart because it is the number a transport's SHAPE decides (one
  handler per byte, per FIFO level, per block), where `busy` also holds
  the thread's own spin on a blocking print. It counts the BODY between
  the two stamps; what the core spends before and after it - the
  hardware entry and exit of a Cortex-M, the crt's dispatch on Hazard3,
  the hardware prologue on a QingKe part - lies outside, counted as idle
  or as thread, and is the family's own documented figure. A handler's
  `isr` is comparable across families; an interrupt's full price is
  `isr` plus that figure.
- `rate` - bytes per second achieved, from `wall`.
- `wire` - bytes per second the configured medium allows. The app
  states it and says why: bits per byte over the line rate for a
  transport, the core's own load/store floor for a memory operation.
- `x` - `wall` over the wire's time, printed by the harness so that no
  measurement is filed unexplained: above about 1.5 it is a finding
  (overview.md), and the document that reports the number carries its
  explanation or lists it under "Not covered yet".

A bench app is NOT a test. `TestBench` still frames it (the menu, the
`ALL:` line `bin/brio` waits for) and a letter's only verdict is that it
ran, with one exception: the ruler's self-check (below) is a verdict,
because a ruler that is wrong makes every other line wrong.

## The ruler

A `CycleRuler` is a free-running count of CORE cycles, wrapping at 2^32,
readable from any context, that states its own rate - the counters
differ by core, and a dynamic clock may stand at any of its rates when
the line is printed. A difference of two reads is exact under 2^32
cycles: one measurement is bounded at about 90 s at 48 MHz and 24 s at
180 MHz, which every operation here is well under.

| family | ruler | what it counts |
|---|---|---|
| SAM C21, STM32G0, RP2040 | `BasicTicker::cycles()` (`cortexm/ticker.hpp`) | SysTick's position in its period composed with the tick count and the pending flag (`util/cycle_count.hpp`) - core cycles |
| STM32F4 | the DWT's cycle counter (`cortexm/dwt.hpp`) | HCLK cycles in one load, the M3+ core unit |
| RP2350, both architectures | `Timer<1>` with its source on clk_sys (`rp2350/timer.hpp`) | clk_sys cycles: one ruler for both instruction sets, as the stratum's `delay_us` is |
| CH32V006, CH32V203 | `Ticker::cycles()` on the STK (`ch32v00x/ticker.hpp`, `ch32vx03/ticker.hpp`) | HCLK cycles |
| AVR DA/DB | two TCBs cascaded at CLK_PER (`avrdx/tcb.hpp`'s `CascadedCounter`) | CLK_PER cycles, 32 bits |

The self-check, letter `r` of every bench app: `delay_us(clock, 1000)`
read on the ruler must be at least hz/1000 and not more than a few per
cent over ("at least, never early", [clock.md](clock.md)); and the
instrument's own cost - a stamp pair, a ruler read, an idle window with
nothing in it - is measured there and printed once per run. Every bench
line carries RAW numbers; the reader subtracts the instrument's cost
where it matters, with the figure from the same run.

## Busy

The core is busy when it is not idle, and the kernel's idle path is one
call the platform owns. `BenchIdle<P, R>` is a platform that forwards
everything to `P` and stamps the ruler around `P::idle()` (and around
`P::idle_until()` exactly where `P` offers it, so the kernel's detection
sees what it would see on `P`): the kernel is templated on the platform
and compiles nothing new, the bench app names the adapter where it would
name `P`.

An idle window measured that way holds the interrupt that ended it: its
handler ran inside the window, so its cycles are not idle and are added
back; a handler that preempted the thread ran inside `wall` and outside
every window, and is already counted. `IsrMeter` is the stamp pair a
bound vector carries, keeping the two apart by asking the adapter
whether a window is open:

    busy = wall - idle_windows + handler_cycles_inside_them

Two seams are stated and not closed: the latency of entering and leaving
a handler (a few cycles either side of the stamps) is counted as idle,
and an interrupt that lands between `P::idle()`'s return and the window's
close is counted in both - a window of a few cycles once per idle turn.
Both are below the instrument's own cost, which letter `r` prints.

The counters are read QUIESCENT: before the operation starts and after
it has completed, when nothing of it is in flight. On a core whose
`atomic_width` is below 4 the app reads them under its platform's
critical section, as it would any 32-bit value a handler writes.

## The protocol

- Sizes: 1, 16, 256 and 4096 bytes for every transport; 1, 2, 16 and 255
  for the I2C tenures the bus's byte counters bound. Memory operations
  at the same four sizes.
- Both columns on one board: the brio side is the bench app; the vendor
  side is built in a scratch tree from the vendor's own sources at a
  pinned revision, with its OWN startup (a vendor's startup may enable
  interrupt nesting or the prefetch where brio's does not - the report
  says what differed), and prints the same lines. The vendor's code is
  never in the tree: it is the oracle of shape and cost
  ([overview.md](overview.md)), read before an engine is written and
  run beside it after.
- The controls a comparison needs are run as columns of their own: the
  instruction fetch (a prefetch off and on, code in flash and in RAM)
  and the clock rate, each stated with the line.
- "We need not win every one; a gap of more than about a fifth is a
  finding to settle" - the rule the two columns are read by.

## What is measured where

The skeleton every `bench_<family>` carries, none of it needing a wire
beyond the console:

- `r` - the ruler's self-check and the instrument's cost.
- `m` - `memcpy` and `memset` at the four sizes: the runtime
  ([runtime.md](runtime.md)) against the core's own load/store floor.
- `p` - a print of the four sizes through the console transport at the
  console's rate: the path every family's print takes, a run at a time
  ([serial.md](serial.md)).
- `t` - the tick's floor: a second of idle with nothing to do, the tick
  handler and the loop's turnaround being all that is busy.

The bus operations (a bus request polled, pumped and engined; a receive
run; an I2C tenure) join the app as the driver they measure is
reworked, each with its own document carrying the numbers.

## The skeleton's numbers

Measured on the bench boards, the console at 115200 8N1 on every family
(one wire row, 11 520 B/s), the memory operations against the core's
own floor (two bytes a cycle for a copy and four for a fill on the
32-bit cores: one word in and one out, or one out, per cycle; one load
of two cycles and one store of one on the AVR). `ruler` and `stamp` are
the instrument's own cost in cycles, `window` the busy cycles of one
idle turn with nothing pending but the tick; `isr` on the print row is
per interrupt, stamps included, the body alone in parentheses; the tick
floor is busy cycles in one idle second.

| family (board, clock) | ruler | stamp | window | memcpy 4096 x | memset 4096 x | print 4096: irq, isr, x | tick floor |
|---|---|---|---|---|---|---|---|
| AVR128DB48 (24 MHz crystal) | 78 | 217 | 355 | 2.36 | 5.02 | 4463 (4096 DRE + 367 ticks), 130 (57), 0.99 | 365 k, 1.52 % |
| SAM C21J18A (48 MHz OSC48M, 2 WS) | 65 | 150 | 259 | 1.43 | 1.63 | 4453 (4096 SERCOM5 + ticks), 131 (87 with the handlers in SRAM), 0.99 | 250 k, 0.52 % |
| STM32G0B1RE (64 MHz PLL, 2 WS, the prefetch and the FIFO on) | 62 | 118 | 213 | 1.41 | 1.58 | 869 (513 USART2 refills + 356 ticks), 205 a refill of up to eight = 41 a character, 1.00 | 207 k, 0.32 % |
| CH32V203C8T6 (144 MHz PLL, zero-wait window) | 32 | 79 | 94 | 1.53 | 1.81 | 4453 (4097 USART1 + 356 ticks), 73 (41), 1.00 | 190 k, 0.13 % |
| STM32F446RE (180 MHz PLL, 5 WS, ART on) | 1 | 27 | 133 | 1.37 | 1.50 | 4453 (4097 USART2 + 356 ticks), 71, 1.00 | 132 k, 0.07 % |
| RP2350, Cortex-M33 (150 MHz PLL, XIP) | 3 | 18 | 156 | 1.12 | 1.24 | 501 (146 FIFO refills of 28 + 355 ticks), 417 a refill = 15 a byte, 0.99 | 154 k, 0.10 % |
| RP2350, Hazard3 (150 MHz PLL, XIP) | 3 | 22 | 70 | 1.39 | 1.53 | 501 (146 refills + ticks), 477 a refill = 17 a byte, 0.99 | 69 k, 0.045 % |

What the rows say, and the two platform columns:

- Every print is wire-bound (`x` = 1.00): the transport's shape costs
  CPU, not time, at a console's rate. On a block with one data register
  and no FIFO (the SERCOM, the F1 lineage's USART, the F4's) that shape
  is one interrupt per byte, as the retrospective review counted, plus
  one (an empty entry from an idle transmitter on the SERCOM); on a
  block with a FIFO it is one interrupt per REFILL - the G0's USART
  fills eight at its empty threshold, the PL011 twenty-eight as its FIFO
  falls through its level, the first FIFO-depth bytes from an idle
  transmitter taking none. The PL011's row was the number of the util
  round: before it, every `write_byte` a full ring refused pended the
  line and the handler found the FIFO full - 474 388 interrupts for 4096
  bytes on the M33, a third of the core for a print the wire bounds; the
  policy is written from the transmit interrupt's measured semantics
  ([../pl011/README.md](../pl011/README.md)).
- The runtime's copy and fill on the Thumb cores run their block as a
  load-multiple/store-multiple turn of 64 bytes ([runtime.md](runtime.md),
  "The block on a Thumb core"): 1.1 to 1.6 times the core's floor at
  4096 bytes, under newlib's own at 256 and 4096 on the M4. The RISC-V
  cores keep the C++ loops, one branch a turn, at 1.4 to 1.8. The AVR's
  row is avr-libc's byte loops (7 and 5 cycles a byte against 3 and 1),
  unchanged. What is left above the floor is the head and the tail of
  each call and the compare a turn - explained, and the gap to newlib at
  16 bytes (the copy two frames deep) a finding of the runtime's.
- The instrument is a tenth to a third of what it measures on the
  cores whose ruler is a ticker's `cycles()`: the tick count composed
  with the counter's position, 32 cycles on the QingKe V4 and 60 to 75
  on the M0+ parts - there the same 36 Thumb instructions fetched from a
  flash behind two wait states, which the SRAM-placed image's 102 stamp
  against 151 proves - and a stamp pair two reads. A one-load ruler (the
  DWT, the RP2350's timer) costs 1 to 3. Every line carries raw numbers;
  the row's own `ruler` and `stamp` are what the reader subtracts.
- STM32G0, the flash prefetch: measured in one image and run with
  PRFTEN off (the silicon's reset state) and on, the instrument's
  straight-line code gains 20 to 33 per cent, the handler bodies 12 to
  20 (the USART's held by three APB accesses the prefetch does not
  touch), the idle second 19, a copy 14 to 25, a fill of 4096 nothing
  (its loop fits the cache); the prints do not move. The NV suites are
  green with it on, their timings within one per cent, so ES0548 2.2.10
  does not bite an image whose code is in one bank - which the link
  imposes. The family's clock task turns it on at init
  ([../stm32g0/clock.md](../stm32g0/clock.md)); the row above is that
  default, and the app's letter `f` keeps printing both columns.
- SAM C21, code in SRAM: the handlers placed in `.ram_text` (the
  linker's input section inside `.data`, zero bytes when unused) run
  the transport a third faster than from the flash behind two wait
  states - the print's interrupt 131 cycles to 87, the loop's
  transmitter 194 to 198 a byte to 129, its receiver 211 to 221 a
  character to 140 - and the tick's by 33 per cent. The placement pays
  because the per-byte path has no call in it (overview.md's three
  rules for a hot path): a placed handler that calls back into the
  flash goes through a veneer.

## Letter d: the engines on each controller

Measured on the bench boards after the DMA round, each line the best of
its runs; `copy` and `fill` by the family's `DmaCopyEngine` at 4096
bytes against one item per cycle of the DMA's clock, the fixed cost the
launch and the completion of one block; `paced` 256 or 1024 words moved
by a timer's request at a known rate; `spi.dma` an engined SPI request
of 256 or 512 frames at the fastest rate the family's host offers, MISO
floating (the time is the wire's; the data is judged by a loop-back
letter where the suite has one, and on the CH32V203 by the SPI's own
transmit CRC), the fixed cost per transaction = wall minus the wire's
time.

| family | copy 4096 x (the DMA's rate) | fill 4096 x | copy fixed cost | paced x | spi.dma x, irq | spi fixed cost per transaction |
|---|---|---|---|---|---|---|
| SAM C21J18A | 2.77 (5 cycles a word) | 2.86 | ~390 cycles (750 with the instrument inside) | 1.00 | 1.04 at 12 MHz (a write-only request on one channel 1.03), 1 | 1416 cycles (was 2365, two interrupts) |
| STM32G0B1RE | 2.73 (5 cycles a word) | 2.73 | ~480 cycles (the instrument inside) | 1.00 | 1.06 at 4 MHz, 1.26 at 16 MHz; 16-bit frames 1.03, 1 | 1084 cycles (was 1291) |
| STM32F446RE | 3.62 (slower than the runtime's memcpy: the engine saves CPU, not time) | 5.11 | 72 cycles (was 1027) | 1.00 | 1.03 at 22.5 MHz; 16-bit frames 1.01, 1 | 606 cycles = 3.4 us (was 13.7 us, two interrupts) |
| CH32V203C8T6 | 6.23 (6 cycles an item, the silicon's) | 6.23 | 241 cycles (was 360) | 1.00 | 1.06 at 36 MHz, 1.03 at 512 frames; 16-bit frames 1.03, 1 | 571 cycles (was 1087, two interrupts): 334 the engines, 130 the Request's copy |
| RP2350, Cortex-M33 | 1.04 | 1.30 (the cell in one SRAM bank, the destination striped over four) | 48 cycles (was 163) | 1.00 | 1.16 at 75 MHz, 1 | 545..559 cycles = 3.7 us (was 5.2, two interrupts) |
| RP2350, Hazard3 | 1.04 | 1.30 | 47 cycles | 1.00 | 1.15 at 75 MHz, 1 | 524..539 cycles = 3.5 us |

What the rows say:

- A block costs what the chapter's restart costs - two to five stores -
  and the engines are no longer the dominant term of a transaction: on
  every family what remained above the wire was the bus host's own
  `start()` - the Request copied into the engine, the pins and the
  compare - which letter e below took down, and the frame gap the PL022
  keeps between frames in mode 0 and 2 (1.5 SCK periods: +19 per cent
  on 8-bit frames, +9 on 16-bit; a display in mode 3 does not pay it,
  and the DCS link's examples carry mode 3).
- A copy by DMA is the controller's own rate, not the bus's: five cycles
  a word on the SAM and the G0, six an item on the CH32V203, and on the
  M4 slower than the core's own load-multiple block - the engine buys
  CPU time there, never wall time. The RP2350's controller is the one
  that copies at the bus's rate.
- A 16-bit frame rides the engines on every family that has them (the
  beat is the frame), one interrupt per transaction where the receive
  block's completion proves the transmit's.
- The paced transfers hold their rate to the ruler's resolution; on the
  RP2350 a block started right after its binding begins 700 to 1200
  cycles late, measured and unexplained (dma.md).
- The RP's "engined SPI at four to nine times the wire" of the
  retrospective review was the FIRST transaction after init: the XIP
  cache filling with the transaction's own code, 126 to 165 lines at 66
  to 82 cycles each (the boot's QMI setting, the same under the SDK),
  nothing of the DMA's. Inside the block brio and the SDK read the same
  number to the hundredth of a cycle - 19.00 and 76.00 cycles a byte in
  mode 0, 16.00 and 64.00 in modes 1 and 3, where the PL022's frame gap
  is SPH's and not the engine's. What is brio's is the NUMBER of lines:
  about 1.15 KB of code per transaction against the SDK's two inlined
  calls, the SPI round's to shrink ([../rp2350/spi.md](../rp2350/spi.md)).

## Letter e: the SPI host above the wire

Measured on the bench boards after the SPI round, each line the best of
its runs with the overrun flag read after every one; `spi.poll` a
polled WRITE of 256 frames (no receive buffer: the display's pixel
path), `spi.poll.rx` the same with one (the shape that keeps as many
frames in flight as the receive side holds under any handler of the
image - a FIFO's depth, two on a two-level buffer, and on a one-deep
register two only above the family's computed rate threshold), `spi.pump`
the write on interrupts, `spi.req` a polled request of one command byte
and two of data at the family's /16 with the select and the D/C on
real pads - the fixed cost = wall minus the wire's time, THE NUMBER OF
THE ROUND, the price of a DCS command -, and the vendor's own polled
full-duplex loop on the same board in a scratch program (ST's
`HAL_SPI_TransmitReceive`, the EVT's 2Lines loop, the pico-sdk's
blocking loop, the data sheet's sequence where no library was read).
The rates named are SCK.

| family (clock) | spi.poll x at /4, /16 | spi.poll.rx x at /4, /16 | spi.pump x at /4, /16; interrupts per 256 | spi.req 3 bytes, fixed cost (before) | the vendor's loop, x at /4, /16 |
|---|---|---|---|---|---|
| SAM C21J18A (48 MHz; 12 and 3 MHz) | 1.06, 1.01 | 1.33, 1.01 (42.6 cycles a character at 12 MHz: the two-flag turn does not fit a 32-cycle character behind two wait states, 39 from SRAM) | 11.76, 2.94; 258 (the handler, 377 cycles a character, outlasts the character at both rates) | 608 (943) | the data sheet's sequence, 1.14, 1.00 (36.6 cycles a character at 12 MHz: the bare loop is not wire-bound there either) |
| STM32G0B1RE (64 MHz; 16 and 4 MHz) | 1.04, 1.01 | 1.23, 1.01 | 5.31, 1.34; 86 (three byte frames an interrupt, the FIFO's capacity) | 399 (998) | HAL 5.29, 1.79 (one in flight and a tick call per turn) |
| STM32F446RE (180 MHz; 22.5 and 5.625 MHz) | 1.01, 1.00 | 1.44 (one in flight below the 237-cycle threshold), 1.00 (two) | 2.57, 1.00 (two in flight from /16); 256 | 251 (517) | HAL 2.97, 1.30 |
| CH32V203C8T6 (144 MHz; 36 and 9 MHz) | 1.02, 1.00 | 1.62 (one in flight), 1.09 | 2.88 (two ahead on a write, 128 interrupts), 1.26; 256 | 280 (468) | the EVT's 2Lines loop 1.23, 1.00 - and it loses a frame in every run at /4 on 8-bit frames |
| CH32V303VCT6 (144 MHz; 36 and 9 MHz; the V4F, ilp32f) | 1.02, 1.00 | 1.36, 1.13 | 2.96 (128 interrupts), 1.25 (205) | 230 | not run on this board |
| RP2350, Cortex-M33 (150 MHz; 37.5 and 9.375 MHz) | 1.02, 1.00 in mode 3 (1.21, 1.19 in mode 0: the block's gap) | 1.02, 1.00 | 1.68, 1.02; 32 | 309 (593) | pico-sdk 1.00 at /4 in mode 3, 1.00 at /16 |
| RP2350, Hazard3 | 1.02, 1.00 | 1.02, 1.00 | 1.69, 1.02; 32 | 301 (621) | pico-sdk 1.19, 1.00 |

What the rows say:

- The polled WRITE is the wire's from the second-fastest rate on every
  family measured (x 1.01 to 1.06 at /4), where the retrospective found
  it at two to seven times the wire: every command phase and a
  display's pixel run pay the wire and a fixed cost of 250 to 400
  cycles a request on the 32-bit cores (600 on the M0+ behind two wait
  states), where 470 to 1000 were.
- The polled READ is bounded by the receive side: where the block has a
  FIFO or a two-level buffer it reads at the wire too; on the one-deep
  F1 lineage (the STM32F4, the CH32V203, the CH32V006) it keeps one
  frame in flight below the family's threshold and pays the turnaround
  every frame - 1.44 to 1.62 at /4 - because two in flight lose a frame
  to any handler longer than the frame time, measured on every family
  that tried it; above the threshold it is the wire's. A loop that
  refills on TXE whenever it can - the EVT's shape - falls behind the
  wire cumulatively at /4 and loses frames with nothing live but the
  tick.
- The pump is the handler's: where a frame outlasts the handler (the
  FIFO families from /16, the one-deep ones from their threshold) the
  bus never idles; faster than that the handler bounds the bus and the
  thread, and the polled style or the engines are the bulk path - each
  family's `dma_min_frames` is that quotient.
- Priming before arming is the rule on every family with a write-ahead:
  the G0, the CH32V203, the PL022 and the SAM each measured a handler
  entering between the first two writes when the line was armed first,
  a frame written twice or a FIFO overrun.
- The vendor's loops are not wire-bound at /4 on any of these cores
  (a tick call per turn in the HAL, a two-flag alternation elsewhere);
  brio's write shape is ahead of every one of them and its read shape
  within a fifth or ahead, the gaps named in each family's SPI document.

## Letter u: the UART transport

Measured on the bench boards after the serial round, each family on a
loop of one of its own UARTs (the instance, the loop and what feeds the
receiver named in the family's serial document): `uart.tx` a run of 256
and 4096 bytes through `write_bulk()`, `uart.rx` a burst of 16 and 256
bytes into the receive path with a consumer draining it, `uart.edge` the
cycles from the burst's last stop bit to the edge leaving the handler -
the contract's "within two frame times" ([serial.md](serial.md)) -, at
115200 and up to the loop's top rate, every vector bound plain (as an
app binds it), counted and metered; `tx_idle()` timed on the pad by the
family's serial suite; the vendor's receive (its library, or the data
sheet's sequence where none exists) on the same board in a scratch
program. `x` is read on the plain binding.

| family (clock; the loop's top rate) | uart.tx x at the top rate, 256 and 4096 | uart.rx 256 x by rate; receive entries for 256 | the edge after the last stop bit | `tx_idle()` after the last stop bit | the core at about 1 Mbaud: transmit, receive (the vendor's receive) | the vendor's receive, x |
|---|---|---|---|---|---|---|
| SAM C21J18A (48 MHz OSC48M, 2 WS; 3 Mbaud) | the engine 1.03, 1.00; the interrupt transmitter 2.17, 2.14 (194 to 198 cycles a byte against a 160-cycle frame; 1.46 from SRAM) | the interrupt receiver 1.00 at 115200, 1.01 at 1 Mbaud, at 3 Mbaud 0.34 entries a character and lossless with no meter on the vectors; the engine 1.00, 1.02, 2.09 at 3 (its plain sender slower than the wire); 256 entries at 1 Mbaud, one completion under the engine | the interrupt receiver 416 to 424 cycles; the engine 656 to 763 at a block's end (1.37 frames at 1 Mbaud), a tail at the owner's ask | within a probe turn (94 to 308 cycles) at 115200, 1 and 3 Mbaud | the transmit engine 1 %, the interrupt receiver 44 to 46 % (32 %) | the data sheet's bare RXC handler: 1.01 at 1 Mbaud, 154 cycles a character against 211 to 221; at 3 Mbaud 241 of 256 lost |
| STM32F446RE (180 MHz PLL, 5 WS, ART on; 5.625 Mbaud) | 1.02 (the engine 1.02), 1.00 (1.01) | the interrupt receiver 1.00, 1.00 and 1.02 at 115200, 1 and 5.625 Mbaud (a hardware overrun in two runs of four at 5.625 beside the bench's meters), the engine 1.00, 1.01 and 1.03; 256 entries for 256, the engine two USART entries a burst and one a half lap | the engine 1.0 frame at 115200 and 1 Mbaud, 1.5 at 5.625 Mbaud | +8 to +23 cycles at 115200 and 1 Mbaud, both transmitters | the transmit engine 0.7 %, the interrupt transmitter 12.5 %; the interrupt receiver 5.8 %, the engine 0.2 % | HAL v1.8.5 ReceiveToIdle_DMA: the edge at the same frame (497 cycles at 5.625 Mbaud against 508), one USART interrupt a burst against two |
| CH32V203C8T6 (144 MHz PLL, zero-wait window; 4.5 Mbaud) | 1.00 (the engine 1.00), 1.00 (1.00) | the interrupt receiver 1.00, 1.00 and 1.01 at 115200, 1 and 4.5 Mbaud, the engine 1.00, 1.00 and 1.01; 256 entries for 256, the engine two USART entries a burst and one a half lap | the engine 1.0, 1.1 and 1.5 frames | +13 to +45 cycles at 115200 (a bit is 1250), within a bit at 1 Mbaud | the transmit engine 0.1 %, the interrupt transmitter 5.6 % (handler cycles); the interrupt receiver 10.6 %, the engine 0.1 % | the EVT's Idle_Recv: the edge at the same frame; 330 to 350 cycles a 16-byte burst against its 324, 434 to 496 a 256-byte one against its 2144 (it copies every block out inside its handler) |
| CH32V006K8U6 (48 MHz; 3 Mbaud transmit, 460800 receive from the host) | the engine 1.03, 1.00; the interrupt transmitter 1.41, 1.40 (one entry a byte outlasts a 160-cycle frame) | the interrupt receiver 0.99 at 115200 and 460800 (the host the sender, n - 1 frames timed), the engine one entry for 256 bytes | the engine 1.1 and 1.2 frames after the last byte landed, at 115200 and 460800 | within a bit at 250000 (the poll's turn longer than a bit at 1 Mbaud) | the transmit engine 3.0 %, the interrupt transmitter 72 % (x 1.08 at 4096 bytes) | not run: the V00x EVT's Idle_Recv handler is the V20x one, byte for byte |
| STM32G0B1RE (64 MHz PLL, 2 WS; 2 Mbaud) | 1.05 (the engine 1.04: the first block's start), 1.00 (1.01) | the paced receiver 1.00, 1.02 and 1.05 at 115200, 1 and 2 Mbaud, the engine 1.00, 1.01 and 1.02; 64 entries for 256, the engine two marks | the paced receiver 1.06 frames at 115200, at 1 Mbaud the tail taken by the level's own entry; the engine's IDLE 1.06 frames and 1.40 to 1.53 at 1 Mbaud | +8 cycles (the interrupt transmitter) and -29 (the engine, inside the poll's resolution) at 9600 baud, a bit 6666 | transmit 19 % (the engine 3.5 %), receive 13 % (17 %: the HAL's FIFO receive) | HAL v1.4.7: its FIFO receive 107 cycles a byte against 84, and the length wanted up front; ReceiveToIdle_DMA's edge 1.04 and 1.34 to 1.49 frames, within 5 per cent |
| RP2040 (125 MHz; 7.8125 Mbaud) | 1.05 (the engine 1.09: the first block's start in a cold image), 1.00 (1.00) | the interrupt receiver 1.00, 1.01 and 1.09 at 115200, 1 and 7.8125 Mbaud, the engine 1.00, 1.01 and 1.07; 21 entries for 256 at 1 Mbaud, the engine two completions | the interrupt receiver 3.17 frames at 115200 (RT), 1.3 to 2.7 at 1 Mbaud; the engine's tail at the owner's ask | 0 to 1 us after it at 9600 baud, a bit 104 us | transmit 2 % (25 cycles a byte), receive 3.4 % (43 a byte); the SDK's polled loop the whole wire time | pico-sdk 2.3.1, polled loops only: 58 cycles a byte read from the FIFO against 43 |
| RP2350, Cortex-M33 (150 MHz; 9.375 Mbaud) | 1.02 (1.07), 1.00 (1.00) | 1.00, 1.01 and 1.10, the engine 1.00, 1.00 and 1.07; 20 entries for 256 at 1 Mbaud | 3.18 frames at 115200, 3.26 at 1 Mbaud; the engine's tail at the owner's ask | 0 to 1 us, a bit 104 us | 1.1 % (17 a byte), 2.1 % (31 a byte) | pico-sdk: 34 cycles a byte against 31 |
| RP2350, Hazard3 (150 MHz; 9.375 Mbaud) | 1.01 (1.03), 1.00 (1.00) | 1.00, 1.00 and 1.03, the engine 1.00, 1.00 and 1.01; 20 entries for 256 at 1 Mbaud | 3.18 and 3.25 frames; the engine's tail at the owner's ask | 0 to 1 us, a bit 104 us | 1.1 % (17 a byte), 1.7 % (25 a byte) | pico-sdk: 29 cycles a byte against 25 |
| AVR128DB48 (24 MHz; 3 Mbaud) | 1.02 to 1.03, 1.00 | 1.00 at 115200 and 1 Mbaud, 1.02 at 2, 3.7 to 6.8 at 3 (the consumer starved); 256 and 255 to 1 Mbaud, about 1.5 frames an entry at 3 | -3 to +2 cycles at 115200; +69 to +77 at 1 to 3 Mbaud, under a frame | +35 to +37 cycles at 115200 (a bit is 208), +47 at 460800 (52), +26 at 1 Mbaud (24) | 25 %, 31 to 35 % (27 %) | the data sheet's sequence: 1.00 at 115200 and 1 Mbaud, 1.01 at 2, 2.9 at 3 |

What the rows say:

- A receiver with no idle flag, no time-out and no FIFO - the AVR's
  chapter 27 - makes its edge on the burst's FIRST byte, and pays the
  silicon's one interrupt a frame: 75 cycles for a frame alone in its
  entry, 30 for each further frame the same entry takes from the
  two-level buffer. That holds the wire to 2 Mbaud at 24 MHz; at 3 Mbaud
  a frame lasts 80 cycles, the handler takes 75 of them, and the
  consumer starves while the hardware loses 1 to 6 frames in 256 - the
  core's limit, which the data sheet's own sequence (65 cycles a frame)
  meets too ([../avrdx/usart.md](../avrdx/usart.md)).
- A one-level receiver whose flags clear by a status read and a data
  read - the F1 lineage: the STM32F4, the CH32V203, the CH32V006 -
  takes its engine's edge from IDLE and never reads the data register:
  the vector's status read and the stream's next read finish each clear,
  so no byte is taken, and because that clear also forgets the IDLE of a
  burst of one frame, the vector takes RXNE once a burst to learn it
  finished - two entries a burst where the HAL takes one, at the same
  frame. The HAL's one entry is its read of DR under DMAR, which can
  take a frame from the stream with no flag raised
  ([../stm32f4/usart.md](../stm32f4/usart.md)).
- A FIFO with a level and a time-out - the STM32G0's FULL USART -
  paces the interrupt receiver at half its depth, a quarter of an entry
  a character, and the time-out at ten bit times delivers the tail
  within two frames; its receive engine's edge is the line's IDLE and
  the ring's half and full marks, the same shape as the HAL's
  ReceiveToIdle and within 5 per cent of its edge
  ([../stm32g0/usart.md](../stm32g0/usart.md)).
- The PL011 paces its receiver at sixteen entries an interrupt - read
  without asking the flag register, the level guaranteeing them - and
  its receive time-out tells the tail after 32 bit periods: 3.2 frames,
  the silicon's fixed figure, above the contract's two. The time-out
  does not rise while a receive engine owns the FIFO (measured on all
  three cores), so under an engine the edge is a run's completion and a
  tail the owner's ask; and a byte-wide beat drops an entry's error
  flags, so a break's zero reaches the ring while the error interrupts
  count it ([../pl011/README.md](../pl011/README.md)). The SDK has
  polled loops only; brio's receiver costs 9 to 26 per cent fewer cycles
  a byte and leaves the core free between its entries.
- The SERCOM has neither an idle flag nor a receiver time-out, so
  under the engine the edge is a block's completion - half the ring -
  and a tail shorter than a block waits for the owner's ask; the idle
  detector a TC could build from the RX pad's edges is declined
  ([../samc21/sercom.md](../samc21/sercom.md)). Its interrupt receiver
  takes every level of the buffer an entry and keeps 3 Mbaud where the
  data sheet's handler, a character an entry, loses nine in ten - at
  37 to 43 per cent more cycles a character than that handler at 1
  Mbaud: the shared vector's question, the RXC re-read that ends the
  loop and the ring's tests, named in the family's document. At 3
  Mbaud it has two frames of margin and no more: the bench's meters on
  every vector cost it a character in most runs from the flash, none
  from SRAM.
- `tx_idle()` turns true after the last stop bit and never before it on
  the pad; at 1 Mbaud on the AVR the poll's own turn, about twenty
  cycles, is most of what the instrument reads.

## Letter i: the I2C host

Measured on the bench boards after the I2C round, each family's host
against the bus partner its desk gives it (the self-link's other
instance served from its own interrupt, a peer board's `twi_peer`, or a
device soldered to the board - the family's document says which):
`i2c.write` and `i2c.read` of 1, 2, 16 and 255 bytes, `i2c.wr` the
register read (one byte written, a repeated START, 1, 2 or 16 read),
`i2c.probe`, the same through each engine, at 100 kHz, 400 kHz and Fm+
where the wire carries it; `wire` the bus's own time for the tenure at
the measured SCL period, so the FIXED COST - wall minus wire, in core
cycles - is the round's number, the price a requester pays for a
tenure beyond the bits; the vendor's own tenure on the same board in a
scratch program.

| family (clock; the partner) | 1-byte write, fixed cost at 400 kHz (before) | register read 1+1, fixed (before) | 255-byte write: x, interrupts | 255-byte read: interrupts (before), handler cycles | busy of a 1-byte write | the vendor | longest host entry (before) |
|---|---|---|---|---|---|---|---|
| AVR128DB48 (24 MHz; the same TWI's client on PC2/PC3, served on the same core) | 509 (599) | 1009 (1112) | 1.12 (1.16), one interrupt a byte - the data sheet's own loop 1.11 | 1.14, smart mode | the plain binding is busy for the tenure, the client's handler on the same core | the data sheet's polled loop: 316 and 808 fixed, the core held the whole tenure | 95 (120); 305 metered |
| ATSAMC21J18A (48 MHz OSC48M, 2 WS; the AVR's `twi_peer`, which stretches every byte about 220 cycles) | 975 through the pump, the plain binding (1205); 1340 metered | - | the engine 1.10 at 254 bytes - the peer's stretch -, three interrupts; the pump 1.24 | the engine one interrupt, 3007 cycles; the pump 103 467 (137 679) | - | the data sheet's loop: about 320 fixed on a one-byte write, the pump's wall about a fifth behind it; within half a per cent on long tenures | 383 |
| STM32F429ZI (180 MHz; the board's STMPE811 on I2C3) | 139 (961), three interrupts (seven) | 389 (816), five (ten) | 1.00 through the pump and the engines | - (the device answers no read a write did not open: reads are register reads) | 708 | HAL: 387 and 1171 fixed; 148 and 153 cycles a byte written and read, against 80 and 99 | 203 |
| CH32V203C8T6 (120 MHz for the letter - the bus clock's 60 MHz ceiling; I2C2 on the self-link, served on the same core) | 1072 (1787), three interrupts (five) | 2006 (2394), five (seven) | 1.00 through the pump (256 interrupts) and the engine (4, the core kept awake under a DMA block: 473 k busy against the pump's 86 k) | - | 1150 | the EVT: 1865 fixed in seven interrupts; 174 and 227 cycles a byte written and read, against 118 and 131 | 164 |
| CH32V006K8U6 (48 MHz; the STM32G0's `twi_peer`) | 1418 (1896), four interrupts (five) | 1967 through the pump (2456) | 1.00 through the pump (264 interrupts) and the engine (9) | 262 through the pump, 9 through the engine | 1759 | WCH's polled master, its library having no interrupt-driven tenure: 766 and 907 fixed, the core held the whole tenure (3103 and 5581 busy) | 190 to 225 |
| STM32G0B1RE (64 MHz; I2C2 on the self-link) | 1147 (1554), one interrupt (two) | 1852 (2377), two interrupts (four) | 1.00 through the pump (260 interrupts, 127 cycles a byte against the HAL's 168) and the engine (6) | 260 through the pump, 5 through the engine | 857 | HAL v1.4.7: 1362 and 2289 fixed | 509, an engined write to an absent address |
| RP2040 (125 MHz; I2C1 on the self-link) | 832 (967), the same at 100 kHz and 1 MHz (1529 at 100 kHz before) | 890 (1011) | 1.00, 31 | 32 (285), 13 378 (54 955) | 802 | pico-sdk 2.3.1, polled: 538 fixed with the core held the whole wire (6680 busy); its 255-byte read 20 834 fixed against 1375 | 439 (574) |
| RP2350, Cortex-M33 (150 MHz; the same) | 474 (699) | 510 (393, which answered at the last byte, a period before the bus was free) | 1.00, 31 | 32 (285), 9007 (32 319) | 583 | pico-sdk: 459 fixed, 7811 busy; its 255-byte read 13 713 fixed against 676 | 298 (353) |
| RP2350, Hazard3 (150 MHz; the same) | 484 (731) | 512 (417) | 1.00, 31 | 32 (285), 8441 (30 597) | 302 | pico-sdk: 486 fixed, 7838 busy | 284 (316) |

What the rows say:

- The DW_apb_i2c host reads at half its receive FIFO and pours read
  commands as the bytes come back, so a tenure of eight bytes or less
  is one interrupt and a 255-byte read 32 where it was 285; every
  tenure ends on its own STOP, and the block's disable - a wait the bus
  timing sets, 1.5 us at 400 kHz and 6 at 100 - is paid only when the
  address or the speed moves, which is what made the fixed cost the
  same at every speed ([../dw_apb_i2c/README.md](../dw_apb_i2c/README.md)).
  The SDK's polled loop is cheaper by 55 per cent on the M0+ for a
  one-byte tenure - no exception entry, no idle wake - and holds the
  core for the whole wire time to get it; on the M33 and the Hazard3
  the two are within a fifth.
- On the F1 lineage's event machine - the STM32F4, the CH32V203, the
  CH32V006 - one interrupt a byte is the silicon's, and the tenure's
  fixed cost is the engine's: the event interrupt raised by `start()`
  and dropped at the completion (the BTF a write's STOP leaves standing
  had re-entered the vector sixteen times a write at 100 kHz on the
  F4), the first two data bytes loaded in the ADDR entry. The F4's
  one-byte write costs 139 cycles beyond the wire, the HAL's 387. The
  wait in `start()` for the last STOP stays - nothing in either chapter
  signals it leaving - and costs about one bit period when a tenure
  follows another at once.
- The SAM C21's SERCOM offers a byte counter (ADDR.LEN) and smart mode:
  an engined read is one interrupt a tenure; a write has no edge at its
  counted end, so LEN = w + 1 makes the MB after byte w the edge that
  sends the STOP or the repeated START - three interrupts, and a
  255-byte write stays the pump's ([../samc21/i2c.md](../samc21/i2c.md)).
  The AVR's TWI holds SCL from each flag until the software answers, so
  a long tenure at 400 kHz runs at the handler's pace - 1.12, the data
  sheet's own loop 1.11 ([../avrdx/twi.md](../avrdx/twi.md)).
- A block with a byte counter and AUTOEND but no FIFO - the STM32G0's
  - takes the pump's one interrupt a byte, and the tenure's fixed cost
  is what is left to cut: the first data byte preloaded into TXDR
  before the START, a read's last byte and a NACK both taken at STOPF,
  the enables written in one store each way - one interrupt for a
  one-byte write where there were two, a fifth under the HAL's
  ([../stm32g0/i2c.md](../stm32g0/i2c.md)).
- On the RP2350's Cortex-M33 a handler's cost from flash moves with
  where the linker puts it - 188 to 365 cycles across three builds of
  one handler, up to six XIP misses a tenure; from SRAM 84. The bench
  runs both I2C vectors from SRAM, before and after alike.

## Not covered yet

Implemented, not bench-verified:

- Letter e on the AVR128DB48 and the CH32V006: the hosts reworked and
  their costs counted in the listings (an AVR polled byte 26 to 30
  cycles against 32 of wire at CLK_PER/4, a CH32V006 3-byte request
  about 300 cycles), their boards not on the desk; one run of each
  bench app's letter e says the measured figures.

- The skeleton on the RP2040 and the CH32V006: the apps build for every
  board type and their vectors read clean in the disassembly, their
  boards not being on the desk; one run of each app's all-key fills
  their rows.
- The instrument's two stated seams (a handler's entry and exit counted
  as idle, an interrupt between the idle call's return and the window's
  close counted twice) stay below its own cost on every family measured,
  the one-load rulers (the DWT, the RP2350's timer) included: the M4's
  idle turn is 133 busy cycles of which 28 the tick's handler, the rest
  the loop's own turn - the seams are inside those, unsplit.
