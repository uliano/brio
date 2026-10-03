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
| SAM C21J18A (48 MHz OSC48M, 2 WS) | 65 | 150 | 259 | 1.43 | 1.63 | 4452 (4096 SERCOM5 + ticks), 168 (111), 0.99 | 250 k, 0.52 % |
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
- SAM C21, code in SRAM: the two handlers placed in `.ram_text` (the
  linker's input section inside `.data`, zero bytes when unused) cut
  the print row's `isr` by 15 per cent as the transport is - its
  per-byte body outlined at `-Os`, so the placed handler calls back into
  the flash through a veneer - and by 35 per cent at every size with the
  whole path inline, the tick's by 33. The placement pays once the
  per-byte path has no call in it (overview.md's three rules for a hot
  path), not before.

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
