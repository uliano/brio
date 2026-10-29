# The benchmark

This page decides the instrument a driver's cost is read with: one
line per operation, one grammar on every family, so that a number
read on one chip means what the same number means on another, and the
vendor's library doing the same operation on the same board prints a
line the same eyes read. A driver's cost is part of what it is
([overview.md, "A driver's cost is part of what it is"](overview.md#target-strata-drivers-by-role-not-a-hal)):
cycles per byte or per frame, interrupts per byte, cycles under the
mask, stated beside the wire's own figure. It is written for an
embedded programmer who wants to read a family's cost tables and know
what each column was measured against, and for one who adds a letter
to a bench. The tables at the end are the skeleton's numbers on every
bench board; each family's documents carry the rest.

Contract and helpers: `util/bench.hpp` (pure, host-tested in
`test/test_bench`). One application per build project,
`bench_<family>`, carries the operations; each family's document
states which of its drivers the application measures and what the
numbers were. A bench application is the apparatus of the measurements
reported here and in the family pages, and is named for that reason.

## The line

    bench <op> n=<bytes> wall=<cycles> busy=<cycles> irq=<count> isr=<cycles> rate=<B/s> wire=<B/s> x=<wall/wire>

- `wall`: ruler cycles from the operation's start to its completion,
  the idle included.
- `busy`: the cycles the core was not idle (the formula below).
- `irq`: the interrupts the operation took, counted in the bound
  vectors.
- `isr`: the cycles spent inside those handlers, a part of `busy`,
  told apart because it is the number a transport's shape decides
  (one handler per byte, per FIFO level, per block), where `busy` also
  holds the thread's own wait on a `print`. It counts the body between
  the two stamps; what the core spends before and after it (the
  hardware entry and exit of a Cortex-M, the startup code's dispatch on
  Hazard3, the hardware prologue on a QingKe part) lies outside,
  counted as idle or as thread, and is the family's own documented
  figure. A handler's `isr` is comparable across families; an
  interrupt's full price is `isr` plus that figure.
- `rate`: bytes per second achieved, from `wall`.
- `wire`: bytes per second the configured medium allows. The
  application states it and says why: bits per byte over the line rate
  for a transport, the core's own load/store floor for a memory
  operation.
- `x`: `wall` over the wire's time, printed by the harness so that no
  measurement is filed unexplained. Above about 1.5 it is a finding
  ([overview.md](overview.md)), and the document that reports the
  number carries its explanation or lists it under "Not covered yet".

A bench application is not a test. `TestBench` still frames it (the
menu, the `ALL:` line `bin/brio` waits for) and a letter's only
verdict is that it ran, with one exception: the ruler's self-check
(below) is a verdict, because a ruler that is wrong makes every other
line wrong.

## The ruler

A `CycleRuler` is a free-running count of core cycles, wrapping at
2^32, readable from any context, that states its own rate: the
counters differ by core, and a dynamic clock may stand at any of its
rates when the line is printed. A difference of two reads is exact
under 2^32 cycles: one measurement is bounded at about 90 s at 48 MHz
and 24 s at 180 MHz, which every operation here is well under.

| family | ruler | what it counts |
|---|---|---|
| SAM C21, STM32G0, RP2040 | `BasicTicker::cycles()` (`cortexm/ticker.hpp`) | SysTick's position in its period composed with the tick count and the pending flag (`util/cycle_count.hpp`): core cycles |
| STM32F4 | the DWT's cycle counter (`cortexm/dwt.hpp`) | HCLK cycles in one load, the M3+ core unit |
| RP2350, both architectures | `Timer<1>` with its source on clk_sys (`rp2350/timer.hpp`) | clk_sys cycles: one ruler for both instruction sets, as the stratum's `delay_us` is |
| CH32V006, CH32V203, CH32V303 | `Ticker::cycles()` on the STK (`qingke/ticker.hpp`) | HCLK cycles |
| AVR DA/DB/EA | two TCBs cascaded at CLK_PER (`avrxt/tcb.hpp`'s `CascadedCounter`) | CLK_PER cycles, 32 bits |

The self-check, letter `r` of every bench application: `delay_us(clock,
1000)` read on the ruler must be at least hz/1000 and not more than a
few per cent over ("at least, never early", [clock.md](clock.md)); and
the instrument's own cost (a stamp pair, a ruler read, an idle window
with nothing in it) is measured there and printed once per run. Every
bench line carries raw numbers; the reader subtracts the instrument's
cost where it matters, with the figure from the same run.

## Busy

The core is busy when it is not idle, and the kernel's idle path is
one call the platform owns. `BenchIdle<P, R>` is a platform that
forwards everything to `P` and stamps the ruler around `P::idle()`
(and around `P::idle_until()` exactly where `P` offers it, so the
kernel's detection sees what it would see on `P`): the kernel is
templated on the platform and compiles nothing new, and the bench
application names the adapter where it would name `P`.

An idle window measured that way holds the interrupt that ended it:
its handler ran inside the window, so its cycles are not idle and are
added back; a handler that preempted the thread ran inside `wall` and
outside every window, and is already counted. `IsrMeter` is the stamp
pair a bound vector carries, keeping the two apart by asking the
adapter whether a window is open:

    busy = wall - idle_windows + handler_cycles_inside_them

Two seams are stated and not closed: the latency of entering and
leaving a handler (a few cycles either side of the stamps) is counted
as idle, and an interrupt that lands between `P::idle()`'s return and
the window's close is counted in both, a window of a few cycles once
per idle turn. Both are below the instrument's own cost in an idle
turn, which letter `r` prints; where a handler re-enters as soon as a
window closes they are not ("Not covered yet").

The counters are read quiescent: before the operation starts and after
it has completed, when nothing of it is in flight. On a core whose
`atomic_width` is below 4 the application reads them under its
platform's critical section, as it would any 32-bit value a handler
writes.

## The protocol

- Sizes: 1, 16, 256 and 4096 bytes for every transport; 1, 2, 16 and
  255 for the I2C tenures the bus's byte counters bound. Memory
  operations at the same four sizes.
- Both columns on one board: the brio side is the bench application;
  the vendor side is built in a scratch tree from the vendor's own
  sources at a pinned revision, with its own startup (a vendor's
  startup may enable interrupt nesting or the prefetch where brio's
  does not; the report says what differed), and prints the same lines.
  The vendor's code is never in the tree: it is the oracle of shape and
  cost ([overview.md](overview.md)), read before an engine is written
  and run beside it after.
- The controls a comparison needs are run as columns of their own: the
  instruction fetch (a prefetch off and on, code in flash and in RAM)
  and the clock rate, each stated with the line.
- "brio need not win every one; a gap of more than about a fifth is a
  finding to settle": the rule the two columns are read by.

## What is measured where

The skeleton every `bench_<family>` carries, none of it needing a wire
beyond the console:

- `r`: the ruler's self-check and the instrument's cost.
- `m`: `memcpy` and `memset` at the four sizes: the runtime
  ([runtime.md](runtime.md)) against the core's own load/store floor;
  then `memcpy` at 60 bytes (a printed line's run) and across
  alignments at every size, `strlen` and `memchr` over strings of 15,
  50 and the largest size, and, where no other letter carries it, the
  copy table a transport's crossover is set from: its byte loop
  against `memcpy` over 1 to 128 bytes, the two ends aligned alike and
  apart, in a letter `c` of its own where letter m's image would not
  fit the family's smallest part (runtime.md, "The copy across
  alignments" and "The scans", has the figures).
- `p`: a print of the four sizes through the console transport at the
  console's rate: the path every family's print takes, a run at a time
  ([serial.md](serial.md)).
- `t`: the tick's floor: a second of idle with nothing to do, the tick
  handler and the loop's turnaround being all that is busy.

The bus operations (a bus request polled, pumped and engined; a
receive run; an I2C tenure) join the application as the driver they
measure is reworked, each with its own document carrying the numbers.
A service whose cost depends on the kernel it runs under is measured in
the suite that runs it: the line writer's in the STM32F446RE's Rubato
suite (letter j, below), against the same line printed straight to the
console by `bench_stm32f4` (its letter l, below).

## The skeleton's numbers

Measured on the bench boards, the console at 115200 8N1 on every
family (one wire row, 11 520 B/s; 11 510 on the CH32V006, whose divisor
gives 115107 baud), the memory operations against the core's own floor
(two bytes a cycle for a copy and four for a fill on the 32-bit cores:
one word in and one out, or one out, per cycle; one load of two cycles
and one store of one on the AVR). `ruler` and `stamp` are the
instrument's own cost in cycles, `window` the busy cycles of one idle
turn with nothing pending but the tick; `isr` on the print row is per
interrupt of the console's vector, stamps included, the body alone in
parentheses (the stamps' or the ruler's share taken off, as the row's
application prints it); the tick floor is busy cycles in one idle
second.

| family (board, clock) | ruler | stamp | window | memcpy 4096 x | memset 4096 x | print 4096: irq, isr, x | tick floor |
|---|---|---|---|---|---|---|---|
| AVR128DB48 (24 MHz crystal) | 78 | 217 | 355 | 2.36 | 5.02 | 4463 (4096 DRE + 367 ticks), 101 (25), 0.99 | 366 k, 1.52 % |
| AVR64EA48 (20 MHz quartz) | 72 | 205 | 337 | 2.34 at 2048 (two buffers of 4096 would be more than the part's 6 KB) | 5.03 at 2048 | 2228 at 2048 (2048 DRE + 180 ticks), 95, 0.99 | 338 k, 1.69 % |
| SAM C21J18A (48 MHz OSC48M, 2 WS) | 65 | 150 | 261 | 1.43 | 1.63 | 4452 (4097 SERCOM5 + 355 ticks), 153 (100 with the handlers in SRAM), 0.99 | 254 k, 0.53 % |
| STM32G0B1RE (64 MHz PLL, 2 WS, the prefetch and the FIFO on) | 62 | 118 | 225 | 1.41 | 1.58 | 871 (515 USART2 refills + 356 ticks), 338 a refill of up to eight = 43 a character, 1.00 | 211 k, 0.33 % |
| CH32V006K8U6 (48 MHz PLL, 2 WS) | 46 | 112 | 255 | 4.48 at 2048 (two buffers of 4096 would be the part's SRAM) | 4.08 | 4454 (4098 USART1 + 356 ticks), 133 (87), 0.99 | 251 k, 0.52 % |
| CH32V203C8T6 (144 MHz PLL, zero-wait window) | 32 | 79 | 114 | 1.53 | 1.81 | 4453 (4097 USART1 + 356 ticks), 71 (39), 1.00 | 116 k, 0.08 % |
| CH32V303VCT6 (144 MHz PLL, zero-wait window; the V4F, ilp32f) | 23 | 62 | 113 | 1.52 | 1.80 | 4453 (4097 USART1 + 356 ticks), 65 (42), 1.00 | 108 k, 0.075 % |
| STM32F446RE (180 MHz PLL, 5 WS, ART on) | 1 | 27 | 81 | 1.41 | 1.50 | 4453 (4097 USART2 + 356 ticks), 66, 1.00 | 80 k, 0.044 % |
| RP2040 (125 MHz PLL, XIP) | 54 | 106 | 236 | 1.39 | 1.55 | 501 (146 FIFO refills + 355 ticks), 670 a refill = 24 a byte, 0.99 | 184 k, 0.15 % |
| RP2350, Cortex-M33 (150 MHz PLL, XIP) | 3 | 18 | 187 | 1.15 | 1.24 | 501 (146 FIFO refills of 28 + 355 ticks), 446 a refill = 16 a byte, 0.99 | 180 k, 0.12 % |
| RP2350, Hazard3 (150 MHz PLL, XIP) | 3 | 22 | 76 | 1.39 | 1.53 | 501 (146 refills + 355 ticks), 470 a refill = 17 a byte, 0.99 | 73 k, 0.048 % |

What the rows say, and the two platform columns:

- Every print is wire-bound (`x` = 1.00): the transport's shape costs
  CPU, not time, at a console's rate. On a block with one data
  register and no FIFO (the SERCOM, the AVR's USART, the F1 lineage's
  USART, the F4's) that shape is one interrupt per byte, plus one (an
  empty entry from an idle transmitter on the SERCOM and the F1
  lineage); on a block with a FIFO it is one interrupt per refill: the
  G0's USART fills eight at its empty threshold, the PL011
  twenty-eight as its FIFO falls through its level, the first
  FIFO-depth bytes from an idle transmitter taking none. The PL011's
  policy is written from the transmit interrupt's measured semantics
  ([the PL011 page](../pl011/README.md)); with every `write_byte` a
  full ring refused pending the line instead, the handler found the
  FIFO full 474 386 times for 4096 bytes on the M33, a third of the
  core for a print the wire bounds.
- The AVR's DRE entry is 101 cycles with its stamps, 25 between them:
  `dre()` flattened, so no ring operation is a call
  ([the AVR USART page](../avrxt/usart.md)). The SAM's console vector
  is its receiver's too: 153 cycles a transmit entry from the flash
  and 100 from SRAM, the transmit path spilling and reloading the flag
  word the receive side's body crowds out of the registers (the
  listing).
- The runtime's copy and fill on the Thumb cores run their block as a
  load-multiple/store-multiple turn of 64 bytes ([runtime.md](runtime.md),
  "The block on a Thumb core"): 1.1 to 1.6 times the core's floor at
  4096 bytes, under newlib's own at 256 and 4096 on the M4. The QingKe
  V4 and Hazard3 cores keep the C++ loops, one branch a turn, at 1.4
  to 1.8; the QingKe V2 at 4.1 and 4.5 against a floor that is its
  bus's figure, not a documented instruction timing (its manual gives
  none): the two-stage pipeline and the loop fetched from flash behind
  two wait states are the rest. The AVR's row is avr-libc's byte loops
  (7 and 5 cycles a byte against 3 and 1). What is left above the floor
  is the head and the tail of each call and the compare a turn,
  explained, and the gap to newlib at 16 bytes (the copy two frames
  deep) a finding of the runtime's.
- The instrument is a tenth to a third of what it measures on the
  cores whose ruler is a ticker's `cycles()`: the tick count composed
  with the counter's position, 23 cycles on the QingKe V4F, 32 on the
  V4B, 46 on the V2 and 54 to 65 on the M0+ parts (there the same 36
  Thumb instructions fetched from a flash behind two wait states or the
  XIP cache, which the SRAM-placed image's 102 stamp against 150
  proves), and a stamp pair two reads. A one-load ruler (the DWT, the
  RP2350's timer) costs 1 to 3. Every line carries raw numbers; the
  row's own `ruler` and `stamp` are what the reader subtracts.
- The idle turn is a few loads and stores on every core, and what it
  costs moves with where the code lies more than with what it is. On
  the STM32F446RE an earlier image, run on the same board beside this
  one, reads 133 and a floor of 132 k where this image reads 81 and
  80 k, the turn's instructions the same in both listings: the 52
  cycles are the turn's lines in the ART, not its code. The RP2350's
  tick vector carries letter e's stretch hook (a load and a test inside
  the stamps, four registers pushed and popped outside them, in every
  tick): its handler 36 cycles on the M33 where it was 30, an earlier
  image run on the same board reading 156 and 153 k again.
- STM32G0, the flash prefetch: measured in one image and run with
  PRFTEN off (the silicon's reset state) and on, the instrument's
  straight-line code gains 17 to 23 per cent, the handler bodies 11 to
  14 (the USART's held by three APB accesses the prefetch does not
  touch), the idle second 15, a copy 11 to 16 below 4096 bytes and 1 at
  4096, a fill of 4096 two (their loops fit the cache); the prints do
  not move. The NV suites are green with it on, their timings within
  one per cent, so ES0548 2.2.10 does not bite an image whose code is
  in one bank, which the link imposes. The family's clock task turns it
  on at init ([the STM32G0 clock page](../stm32g0/clock.md)); the row
  above is that default, and the application's letter `f` keeps
  printing both columns.
- SAM C21, code in SRAM: the handlers placed in `.fast_text` (the
  linker's input section inside `.data`, zero bytes when unused) run
  the transport a third faster than from the flash behind two wait
  states (the print's interrupt 149 cycles to 100, the loop's
  transmitter 189 a byte to 124, its receiver 203 a character to 135)
  and the tick's 40 to 28. The placement pays because the per-byte
  path has no call in it (overview.md's three rules for a hot path): a
  placed handler that calls back into the flash goes through a veneer.

## Letter d: the engines on each controller

Measured on the bench boards, each line the best of its runs; `copy`
and `fill` by the family's `DmaCopyEngine` at 4096 bytes (2048 on the
CH32V006, whose SRAM holds no two of 4096) against one item per cycle
of the DMA's clock, the fixed cost the launch and the completion of one
block; `paced` 256 or 1024 words moved by a timer's request at a known
rate; `spi.dma` an engined SPI request of 256 frames at the fastest
rate the family's host offers, MISO floating (the time is the wire's;
the data is judged by a loop-back letter where the suite has one, and
on the CH32V203 and the CH32V303 by the SPI's own transmit CRC), the
fixed cost per transaction = wall minus the wire's time. The rates
named are SCK.

| family | copy 4096 x (the DMA's rate) | fill 4096 x | copy fixed cost | paced x | spi.dma x, irq | spi fixed cost per transaction |
|---|---|---|---|---|---|---|
| STM32G0B1RE | 2.73 (5 cycles a word) | 2.73 | ~487 cycles (the instrument inside) | 1.00 | 1.06 at 8 MHz, 1.26 at 32 MHz; 16-bit frames 1.03, 1 | 1083 cycles (was 1291) |
| STM32F446RE | 3.62 (slower than the runtime's memcpy: the engine saves CPU, not time) | 5.11 | 72 cycles (was 1027) | 1.00 | 1.04 at 22.5 MHz; 16-bit frames 1.01, 1 | 652 to 664 cycles = 3.7 us (was 13.7 us, two interrupts) |
| CH32V006K8U6 | 6.82 at 2048 (6 cycles an item from SRAM, 8 from the flash) | 8.42 (8 cycles an item) | ~500 cycles (the instrument inside) | 1.00 | 1.10 at 12 MHz (32.0 cycles a byte, the wire's), 1.02 at 3 MHz; 16-bit frames 1.05, 1 | 864 cycles (871 in 16-bit frames) |
| CH32V203C8T6 | 6.23 (6 cycles an item, the silicon's) | 6.23 | 245 cycles (was 360) | 1.00 | 1.06 at 36 MHz; 16-bit frames 1.03, 1 | 514 cycles (was 1087, two interrupts): 332 the launch |
| CH32V303VCT6 | 6.28 (6 cycles an item) | 6.25 | 298 cycles | 1.00 | 1.07 at 36 MHz; 16-bit frames 1.03, 1 | 595 cycles: 397 the launch |
| CH32X035F8U6 (48 MHz HSI, 2 WS) | 6.56 (6 cycles an item from SRAM, 11.6 from the flash) | 6.48 | ~890 cycles (the instrument inside, the core held awake by the console's ring) | 1.00 (1024 bytes paced by a USART's transmitter at 1 Mbaud: the stratum has no timer driver) | - (not measured in this grammar: the host's costs are in [the CH32X035 SPI page](../ch32x035/spi.md)) | - |
| RP2040 | 1.38 (the bus's rate; the completion's interrupt in the wall) | 1.63 | ~460 cycles to the completion's interrupt (the instrument inside) | 1.04 | 1.30 at 31.25 MHz (38 cycles a byte: the block's mode-0 gap); 16-bit frames 1.15, 1 | 1021 cycles = 8.2 us |
| RP2350, Cortex-M33 | 1.04 | 1.30 (the cell in one SRAM bank, the destination striped over four) | 49 cycles (was 163) | 1.00 | 1.32 at 75 MHz (19 cycles a byte: the mode-0 gap); 16-bit frames 1.16, 1 | 562 cycles over the wire and its gap = 3.7 us (was 5.2, two interrupts) |
| RP2350, Hazard3 | 1.04 | 1.29 | 47 cycles | 1.00 | 1.30 at 75 MHz; 16-bit frames 1.15, 1 | 494 cycles over the wire and its gap = 3.3 us |

What the rows say:

- A block costs what the chapter's restart costs, two to five stores,
  and the engines are not the dominant term of a transaction: on every
  family what remains above the wire is the bus host's own `start()`
  (the `Request` copied into the engine, the pins and the compare),
  which letter e below takes down, and the frame gap the PL022 keeps
  between frames in modes 0 and 2 (1.5 SCK periods: +19 per cent on
  8-bit frames, +9 on 16-bit; a display in mode 3 does not pay it, and
  the DCS link's examples carry mode 3).
- A copy by DMA is the controller's own rate, not the bus's: five
  cycles a word on the G0, six an item on the CH32V203, the CH32V303
  and the CH32V006 (eight from the CH32V006's flash), and on the M4
  slower than the core's own load-multiple block, so the engine buys
  CPU time there, never wall time. The RP2040's and the RP2350's
  controllers copy at the bus's rate.
- A 16-bit frame rides the engines on every family that has them (the
  beat, the unit a transfer moves, is the frame), one interrupt per
  transaction
  where the receive block's completion proves the transmit's.
- The paced transfers hold their rate to the ruler's resolution; on
  the RP2350 a block started right after its binding begins 700 to
  1200 cycles late, measured and unexplained ([dma.md](dma.md)); on
  the RP2040 the block runs 4 per cent long, its intervals 87 to 628
  cycles against 400 ("Not covered yet").
- An engined SPI transaction on the RP families at four to nine times
  the wire is the first transaction after init: the XIP cache filling
  with the transaction's own code, 126 to 165 lines at 66 to 82 cycles
  each (the boot's QMI setting, the same under the SDK), nothing of the
  DMA's: 5743 cycles for 16 frames on the M33 against 866 warm. Inside
  the block brio and the SDK read the same number to the hundredth of
  a cycle, 19.00 and 76.00 cycles a byte in mode 0, 16.00 and 64.00 in
  modes 1 and 3, where the PL022's frame gap is SPH's and not the
  engine's. What is brio's is the number of lines: about 1.15 KB of
  code per transaction against the SDK's two inlined calls, a size
  still to shrink ([the RP2350 SPI page](../rp2350/spi.md)).
- The engined transaction's fixed cost moves with the host's `start()`
  and the platform's wake: the CH32V203's 571 cycles became 514 when
  its idle path took one turn per wake (the second turn a stale event
  made is gone from every completion); the STM32F446RE's 558 are 652
  to 664 with the hold-off threshold `start()` derives and the image's
  placement, not split.

## Letter e: the SPI host above the wire

Measured on the bench boards, each line the best of its runs with the
overrun flag read after every one; `spi.poll` a polled write of 256
frames (no receive buffer: the display's pixel path), `spi.poll.rx`
the same with one (the shape that keeps as many frames in flight as
the receive side holds under any handler of the image: a FIFO's depth,
two on a two-level buffer, and on a one-deep register two only above
the family's computed rate threshold), `spi.pump` the write on
interrupts, `spi.req` a polled request of one command byte and two of
data at the family's /16 with the select and the D/C on real pads (the
fixed cost = wall minus the wire's time, the price of a DCS command),
and the vendor's own polled full-duplex loop on the same board in a
scratch program (ST's `HAL_SPI_TransmitReceive`, the EVT's 2Lines
loop, the pico-sdk's blocking loop, the data sheet's sequence where no
library was read). The rates named are SCK.

| family (clock) | spi.poll x at /4, /16 | spi.poll.rx x at /4, /16 | spi.pump x at /4, /16; interrupts per 256 | spi.req 3 bytes, fixed cost (before) | the vendor's loop, x at /4, /16 |
|---|---|---|---|---|---|
| AVR128DB48 (24 MHz; 6 and 1.5 MHz) | 1.09, 1.02 (two cycles a byte the block's, beyond the wire) | 1.09, 1.02 | 7.17, 1.79; 130 (two bytes an entry; the handler, 230 cycles a byte, bounds both rates) | 286, the stopwatch's 65 inside | the data sheet's buffer-mode sequence 1.09, 1.01 |
| AVR64EA48 (20 MHz; 5 and 1.25 MHz) | 1.09, 1.02 (the DB's cycles a byte, to the cycle) | 1.09, 1.02 | 7.00, 1.75; 131 (the handler, 223 cycles a byte, bounds both rates) | 280, the stopwatch's 60 inside | not run on this board |
| SAM C21J18A (48 MHz; 12 and 3 MHz) | 1.05, 1.01 | 1.38, 1.01 (44 cycles a character at 12 MHz: the two-flag turn does not fit a 32-cycle character behind two wait states) | 12.64, 3.16; 258 (the handler, 404 cycles a character, outlasts the character at both rates) | 568 (943) | the data sheet's sequence, 1.14, 1.00 (36.6 cycles a character at 12 MHz: the bare loop is not wire-bound there either) |
| STM32G0B1RE (64 MHz; 16 and 4 MHz) | 1.04, 1.01 | 1.17, 1.01 | 5.31, 1.34; 86 (three byte frames an interrupt, the FIFO's capacity) | 384 (998) | HAL 5.29, 1.79 (one in flight and a tick call per turn) |
| STM32F446RE (180 MHz; 22.5 and 5.625 MHz) | 1.01, 1.00 | 1.60 (one in flight below the 236-cycle threshold), 1.00 (two) | 2.61, 1.00 (two in flight from /16); 256 | 243 (517) | HAL 2.97, 1.30 |
| CH32V006K8U6 (48 MHz; 12 and 3 MHz) | 1.03, 1.00 | 1.80 (one in flight), 1.17 (one in flight below /64) | 8.32, 2.08 (the handler, 265 cycles a frame, bounds both rates; the wire's from /64); 257 | 387 | not run on this board |
| CH32V203C8T6 (144 MHz; 36 and 9 MHz) | 1.02, 1.00 | 1.61 (one in flight), 1.09 | 2.88 (two ahead on a write, 128 interrupts), 1.22; 256 | 263 (468) | the EVT's 2Lines loop 1.23, 1.00, and it loses a frame in every run at /4 on 8-bit frames |
| CH32V303VCT6 (144 MHz; 36 and 9 MHz; the V4F, ilp32f) | 1.02, 1.00 | 1.36, 1.13 | 2.57 (128 interrupts), 1.11 (256) | 235 | not run on this board |
| RP2040 (125 MHz; 31.25 and 7.8125 MHz) | 1.13, 1.00 in mode 3 (1.22, 1.19 in mode 0: the block's gap) | 1.19, 1.00 in mode 3 | 2.76, 1.04 in mode 3; 32 (51 at /16) | 462 | pico-sdk ([the RP2040 SPI page](../rp2040/spi.md)) |
| RP2350, Cortex-M33 (150 MHz; 37.5 and 9.375 MHz) | 1.02, 1.00 in mode 3 (1.21, 1.19 in mode 0: the block's gap) | 1.02, 1.00 | 1.69, 1.02; 32 (52 at /16) | 312 (593) | pico-sdk 1.00 at /4 in mode 3, 1.00 at /16 |
| RP2350, Hazard3 | 1.02, 1.00 | 1.02, 1.00 | 1.65, 1.02; 32 (52 at /16) | 277 (621) | pico-sdk 1.19, 1.00 |

What the rows say:

- The polled write is the wire's from the second-fastest rate on every
  family measured but two (x 1.01 to 1.05 at /4): the AVR's 1.09 is
  the block's two cycles a byte, the data sheet's own loop the same
  ([the AVR SPI page](../avrxt/spi.md)), and the RP2040's 1.13 in mode
  3 its loop's 35 cycles a frame against 32 on the M0+. Every command
  phase and a display's pixel run pay the wire and a fixed cost of 235
  to 390 cycles a request on the 32-bit cores, 460 on the RP2040 and
  570 on the SAM behind two wait states, 286 on the AVR.
- The polled read is bounded by the receive side: where the block has
  a FIFO or a two-level buffer it reads at the wire too; on the
  one-deep F1 lineage (the STM32F4, the CH32V203, the CH32V303, the
  CH32V006) it keeps one frame in flight below the family's threshold
  and pays the turnaround every frame, 1.36 to 1.80 at /4, because two
  in flight lose a frame to any handler longer than the frame time,
  measured on every family that tried it; above the threshold it is
  the wire's. A loop that refills on TXE whenever it can (the EVT's
  shape) falls behind the wire cumulatively at /4 and loses frames
  with nothing live but the tick.
- The pump is the handler's: where a frame outlasts the handler (the
  FIFO families from /16, the one-deep ones from their threshold) the
  bus never idles; faster than that the handler bounds the bus and the
  thread, and the polled style or the engines are the bulk path; each
  family's `dma_min_frames` is that quotient.
- Priming before arming is the rule on every family with a
  write-ahead: the G0, the CH32V203, the PL022 and the SAM each
  measured a handler entering between the first two writes when the
  line was armed first, a frame written twice or a FIFO overrun.
- The vendor's loops are not wire-bound at /4 on any of these cores (a
  tick call per turn in the HAL, a two-flag alternation elsewhere);
  brio's write shape is ahead of every one of them and its read shape
  within a fifth or ahead, the gaps named in each family's SPI
  document.
- The request's fixed cost fell with the pin edges made inline
  (`PinRef::set/clear`): the SAM's 608 to 568, the CH32V203's 280 to
  263, the G0's 399 to 384. On the RP2350 the host's place is the
  program's (`util/placement.hpp`): warm, the flash and SRAM read
  within the call's few cycles, and what SRAM buys is the cold request
  ([the RP2350 platform page](../rp2350/platform.md), "A handler in
  SRAM"). The SAM's pump reads 12.64 and 3.16 because the bench's
  SERCOM1 vector serves letter u's loop too: its dispatch and a larger
  prologue stand in front of every pumped character (404 cycles a
  character, 377 with the pump alone on the vector).

## Letter u: the UART transport

Measured on the bench boards, each family on a loop of one of its own
UARTs (the instance, the loop and what feeds the receiver named in the
family's serial document): `uart.tx` a run of 256 and 4096 bytes
through `write_bulk()`, `uart.rx` a burst of 16 and 256 bytes into the
receive path with a consumer draining it, `uart.edge` the cycles from
the burst's last stop bit to the edge leaving the handler (the
contract's "within two frame times", [serial.md](serial.md)), at
115200 and up to the loop's top rate, every vector bound plain (as an
application binds it), counted and metered; where the transport has a
receive engine, `uart.harvest` the masked window of the owner's publish
verb in its three states (nothing landed, a part of the run published,
the run ended and re-armed inside the window); `tx_idle()` timed on the
pad by the family's serial suite; the vendor's receive (its library,
or the data sheet's sequence where none exists) on the same board in a
scratch program. `x` is read on the plain binding; the core's share is
the transport's busy cycles for a transmit and its handler cycles for
a receive, at the rate named.

| family (clock; the loop's top rate) | uart.tx x at the top rate, 256 and 4096 | uart.rx 256 x by rate; receive entries for 256 | the edge after the last stop bit | `tx_idle()` after the last stop bit | the core at about 1 Mbaud: transmit, receive (the vendor's receive) | the vendor's receive, x |
|---|---|---|---|---|---|---|
| SAM C21J18A (48 MHz OSC48M, 2 WS; 3 Mbaud) | the engine 1.02, 1.00; the interrupt transmitter 2.05, 2.03 (189 cycles an entry against a 160-cycle frame; 1.40 from SRAM) | the interrupt receiver 1.00 at 115200, 1.01 at 1 Mbaud, at 3 Mbaud 0.33 entries a character and one character lost with the meters on the vectors from the flash, the skip after it taking the rest of the burst, so the 256 never arrive (lossless from SRAM, 1.09); 255 entries at 1 Mbaud | the interrupt receiver 424 cycles | within a probe turn (94 to 308 cycles) at 115200, 1 and 3 Mbaud | the transmit engine 1 %, the interrupt receiver 46 % (32 %) | the data sheet's bare RXC handler: 1.01 at 1 Mbaud, 154 cycles a character against 223; at 3 Mbaud 241 of 256 lost |
| STM32F446RE (180 MHz PLL, 5 WS, ART on; 5.625 Mbaud) | 1.02 (the engine 1.02), 1.00 (1.01) | the interrupt receiver 1.00, 1.00 and 1.02 at 115200, 1 and 5.625 Mbaud, the engine 1.00, 1.01 and 1.03; 256 entries for 256, the engine two USART entries a burst and one a half lap | the engine 1.0 frame at 115200, 1.1 at 1 Mbaud, 1.5 at 5.625 Mbaud | +8 to +23 cycles at 115200 and 1 Mbaud, both transmitters | the transmit engine 0.6 %, the interrupt transmitter 11.6 %; the interrupt receiver 5.6 %, the engine 0.2 % | HAL v1.8.5 ReceiveToIdle_DMA: the edge at the same frame (497 cycles at 5.625 Mbaud against 502), one USART interrupt a burst against two |
| CH32V203C8T6 (144 MHz PLL, zero-wait window; 4.5 Mbaud) | 1.00 (the engine 1.00), 1.00 (1.00) | the interrupt receiver 1.00, 1.00 and 1.01 at 115200, 1 and 4.5 Mbaud, the engine 1.00, 1.00 and 1.01; 256 entries for 256, the engine two USART entries a burst and one a half lap | the engine 1.0, 1.1 and 1.5 frames | +13 to +45 cycles at 115200 (a bit is 1250), within a bit at 1 Mbaud | the transmit engine 0.1 %, the interrupt transmitter 5.6 %; the interrupt receiver 10.4 %, the engine 0.1 % | the EVT's Idle_Recv: the edge at the same frame; 330 to 350 cycles a 16-byte burst against its 324, 434 to 496 a 256-byte one against its 2144 (it copies every block out inside its handler) |
| CH32V303VCT6 (144 MHz PLL, zero-wait window, the V4F; 4.5 Mbaud) | 1.00 (the engine 1.00), 1.00 (1.00) | the interrupt receiver 1.00, 1.00 and 1.01 at 115200, 1 and 4.5 Mbaud, the engine 1.00, 1.00 and 1.01; 256 entries for 256 | the engine 1.0, 1.1 and 1.5 frames | not measured on this board | the transmit engine 0.1 %, the interrupt transmitter 5.1 %; the interrupt receiver 8.6 %, the engine 0.1 % | not run on this board |
| CH32V006K8U6 (48 MHz; 3 Mbaud transmit, 460800 receive from the host) | the engine 1.03, 1.00; the interrupt transmitter 1.46, 1.46 (one entry a byte outlasts a 160-cycle frame) | the interrupt receiver 0.99 at 115200 and 460800 (the host the sender, n - 1 frames timed), the engine one entry for 256 bytes | the engine 1.1 and 1.2 frames after the last byte landed, at 115200 and 460800 | within a bit at 250000 (the poll's turn longer than a bit at 1 Mbaud) | the transmit engine 2.3 %, the interrupt transmitter 73 % (x 1.09 at 4096 bytes) | not run: the V00x EVT's Idle_Recv handler is the V20x one, byte for byte |
| STM32G0B1RE (64 MHz PLL, 2 WS; 2 Mbaud) | 1.05 (the engine 1.04: the first block's start), 1.00 (1.01) | the paced receiver 1.00, 1.02 and 1.05 at 115200, 1 and 2 Mbaud, the engine 1.00, 1.00 and 1.02; 64 entries for 256, the engine two marks | the paced receiver 1.05 frames at 115200, at 1 Mbaud the tail taken by the level's own entry; the engine's IDLE 1.06 frames and 1.46 at 1 Mbaud | +8 cycles (the interrupt transmitter) and -29 (the engine, inside the poll's resolution) at 9600 baud, a bit 6666 | transmit 20 % (the engine 3.5 %), receive 15 % (17 %: the HAL's FIFO receive) | HAL v1.4.7: its FIFO receive 107 cycles a byte against 96, and the length wanted up front; ReceiveToIdle_DMA's edge 1.04 and 1.34 to 1.49 frames, within 5 per cent |
| RP2040 (125 MHz; 7.8125 Mbaud) | 1.06 (the engine 1.25: the first block's start in a cold image), 1.00 (1.02) | the interrupt receiver 1.00, 1.01 and 1.12 at 115200, 1 and 7.8125 Mbaud, the engine 1.00, 1.01 and 1.09; 21 entries for 256 at 1 Mbaud, the engine two completions | the interrupt receiver 3.18 frames at 115200 (RT), at 1 Mbaud the 17 bytes taken by the level's own entry; the engine's tail at the owner's ask | 0 to 1 us after it at 9600 baud, a bit 104 us | transmit 2.0 % (25 cycles a byte), receive 3.7 % (46 a byte); the SDK's polled loop the whole wire time | pico-sdk 2.3.1, polled loops only: 58 cycles a byte read from the FIFO against 46 |
| RP2350, Cortex-M33 (150 MHz; 9.375 Mbaud) | 1.01 (1.06), 1.00 (1.00) | 1.00, 1.00 and 1.05, the engine 1.00, 1.00 and 1.05; 21 entries for 256 at 1 Mbaud | 3.20 frames at 115200, 3.37 at 1 Mbaud; the engine's tail at the owner's ask | 0 to 1 us, a bit 104 us | 1.1 % (17 a byte), 2.1 % (27 a byte) | pico-sdk: 34 cycles a byte against 27 |
| RP2350, Hazard3 (150 MHz; 9.375 Mbaud) | 1.00 (1.03), 1.00 (1.00) | 1.00, 1.00 and 1.05, the engine 1.00, 1.00 and 1.04; 21 entries for 256 at 1 Mbaud | 3.19 and 3.03 frames; the engine's tail at the owner's ask | 0 to 1 us, a bit 104 us | 1.2 % (17 a byte), 1.9 % (27 a byte) | pico-sdk: 29 cycles a byte against 27 |
| AVR128DB48 (24 MHz; 3 Mbaud) | 1.03, 1.00 | 1.00 at 115200 and 1 Mbaud, 1.04 at 2, 3.9 at 3 (the consumer starved, below); 256 and 254 to 1 Mbaud | -2 to 0 cycles at 115200; +69 to +78 at 1 to 3 Mbaud, under a frame | +35 to +37 cycles at 115200 (a bit is 208), +47 at 460800 (52), +26 at 1 Mbaud (24) | 25 %, 31 to 35 % (27 %) | the data sheet's sequence: 1.00 at 115200 and 1 Mbaud, 1.01 at 2, 2.9 at 3 |
| AVR64EA48 (20 MHz; 2.5 Mbaud) | 1.03, 1.01 | 1.00 at 115200 and 1 Mbaud, 1.08 at 2, 1.69 at 2.5 (the consumer starved, as the DB's at 3); 256 and 257 to 1 Mbaud | +10 cycles at 115200; +70 to +74 at 1 to 2.5 Mbaud, under a frame | +20 cycles at 115200 (a bit is 174), +48 at 460800 (44) | the DB's cycles: a DRE entry 95 metered, a lone frame's receive entry 107, against a 200-cycle frame | not run on this board |

What the rows say:

- A receiver with no idle flag, no time-out and no FIFO (the AVR's
  chapter 27) makes its edge on the burst's first byte, and pays the
  silicon's one interrupt a frame: 75 cycles for a frame alone in its
  entry, 30 for each further frame the same entry takes from the
  two-level buffer. That holds the wire to 2 Mbaud at 24 MHz; at
  3 Mbaud a frame lasts 80 cycles, the handler takes 75 of them, and
  the consumer starves, the core's limit, which the data sheet's own
  sequence (65 cycles a frame) meets too
  ([the AVR USART page](../avrxt/usart.md)). A starved consumer meets
  the receive ring's skips besides: a look after a frame the full ring
  refused discards what the ring holds, so 256 bytes consumed take 3.9
  times their wire time of a stream that does not stop. With the
  meters on the vector it outlasts a frame at 2 and 3 Mbaud, a
  hardware overrun falls between every two looks, every look skips,
  and nothing is handed out at all ([ring.md](ring.md), "What a skip
  costs the stream").
- A one-level receiver whose flags clear by a status read and a data
  read (the F1 lineage: the STM32F4, the CH32V203, the CH32V303, the
  CH32V006) takes its engine's edge from IDLE and never reads the data
  register: the vector's status read and the stream's next read
  finish each clear, so no byte is taken, and because that clear also
  forgets the IDLE of a burst of one frame, the vector takes RXNE once
  a burst to learn it finished, two entries a burst where the HAL
  takes one, at the same frame. The HAL's one entry is its read of DR
  under DMAR, which can take a frame from the stream with no flag
  raised ([the STM32F4 USART page](../stm32f4/usart.md)).
- A FIFO with a level and a time-out (the STM32G0's full USART) paces
  the interrupt receiver at half its depth, a quarter of an entry a
  character, and the time-out at ten bit times delivers the tail
  within two frames; its receive engine's edge is the line's IDLE and
  the ring's half and full marks, the same shape as the HAL's
  ReceiveToIdle and within 5 per cent of its edge
  ([the STM32G0 USART page](../stm32g0/usart.md)). Its receiver's 96
  cycles a byte are the bench vector's: one vector serves the loop's
  four transports, and in that one function gcc reloads the receive
  ring's address from the literal pool at every character (the release
  listing), where a console's vector, which binds one, keeps it in a
  register and drains a character in the plain Ring's 23 instructions.
  The receiver's body in a function of its own, called from the same
  vector, reads 94 cycles a byte for the skipping ring and 95 for a
  plain Ring, in one session.
- The PL011 paces its receiver at sixteen entries an interrupt (read
  without asking the flag register, the level guaranteeing them) and
  its receive time-out tells the tail after 32 bit periods: 3.2 frames,
  the silicon's fixed figure, above the contract's two. The time-out
  does not rise while a receive engine owns the FIFO (measured on all
  three cores), so under an engine the edge is a run's completion and
  a tail the owner's ask; and a byte-wide beat drops an entry's error
  flags, so a break's zero reaches the ring while the error interrupts
  count it ([the PL011 page](../pl011/README.md)). The SDK has polled
  loops only; brio's receiver costs 7 to 21 per cent fewer cycles a
  byte, and leaves the core free between its entries. The level loop
  is the plain Ring's instruction for instruction on every core (20 on
  the Cortex-M0+), and on the RP2040 the figure moves by up to 4 cycles
  a byte between two runs of one image: the XIP cache's state at the
  burst.
- The SERCOM's receiver is its interrupt receiver alone (the family's
  DMA serves its transmitter only,
  [the SAM C21 SERCOM page](../samc21/sercom.md)). It takes every level
  of the buffer an entry and keeps 3 Mbaud where the data sheet's
  handler, a character an entry, loses nine in ten, at 45 per cent
  more cycles a character than that handler at 1 Mbaud: the shared
  vector's question, the RXC re-read that ends the loop and the ring's
  tests, named in the family's document. At 3 Mbaud it has two frames
  of margin and no more: the bench's meters on every vector cost it a
  character from the flash, none from SRAM.
- On the QingKe parts an engine's share of the core is what its wakes
  cost, and the idle path takes one turn per wake: the CH32V006's
  transmit engine is 2.3 per cent of the core at 1 Mbaud, 3.0 when a
  stale event made each wake two turns.
- `tx_idle()` turns true after the last stop bit and never before it
  on the pad; at 1 Mbaud on the AVR the poll's own turn, about twenty
  cycles, is most of what the instrument reads.

## Letter i: the I2C host

Measured on the bench boards, each family's host against the bus
partner its desk gives it (the self-link's other instance served from
its own interrupt, a peer board's `twi_peer`, or a device soldered to
the board; the family's document says which): `i2c.write` and
`i2c.read` of 1, 2, 16 and 255 bytes, `i2c.wr` the register read (one
byte written, a repeated START, 1, 2 or 16 read), `i2c.probe`, the same
through each engine, at 100 kHz, 400 kHz and Fm+ where the wire
carries it; `wire` the bus's own time for the tenure at the measured
SCL period, so the fixed cost (wall minus wire, in core cycles) is the
price a requester pays for a tenure beyond the bits; the vendor's own
tenure on the same board in a scratch program. The busy column is the
400 kHz one, every meter's, the partner's handlers included where they
run on the same core.

| family (clock; the partner) | 1-byte write, fixed cost at 400 kHz (before) | register read 1+1, fixed (before) | 255-byte write: x, interrupts | 255-byte read: interrupts (before), handler cycles | busy of a 1-byte write | the vendor | longest host entry (before) |
|---|---|---|---|---|---|---|---|
| AVR128DB48 (24 MHz; the same TWI's client on PC2/PC3, served on the same core, SDASETUP four clocks) | 509 (599) | 1003 (1112) | 1.12 (1.16), one interrupt a byte; the data sheet's own loop 1.11 | 1.14, smart mode | 1056 metered; the plain binding is busy for the tenure, the client's handler on the same core | the data sheet's polled loop: 316 and 808 fixed, the core held the whole tenure | 95 (120); 305 metered |
| AVR64EA48 (20 MHz; the same TWI's client on PC2/PC3, served on the same core, SDASETUP eight clocks) | 406 bare | 703 bare (the 1 + 1 register read, smart mode) | 1.21 bare, one interrupt a byte | 1.23, smart mode | 998 metered; the plain binding is busy for the tenure | not run on this board | not taken on this board |
| ATSAMC21J18A (48 MHz OSC48M, 2 WS; the AVR's `twi_peer`, which stretches every byte about 220 cycles) | 978 through the pump, the plain binding (1205); 1341 metered | 1371 plain, 1835 metered | the pump 1.24 | the pump 104 575 (137 679) | 1505 metered | the data sheet's loop: about 320 fixed on a one-byte write, the pump's wall about a fifth behind it; within half a per cent on long tenures | 383 |
| STM32F429ZI (180 MHz; the board's STMPE811 on I2C3; the F429's figures, its board not on the desk) | 139 (961), three interrupts (seven) | 389 (816), five (ten) | 1.00 through the pump and the engines | - (the device answers no read a write did not open: reads are register reads) | 708 | HAL: 387 and 1171 fixed; 148 and 153 cycles a byte written and read, against 80 and 99 | 203 |
| STM32F446RE (180 MHz; FMPI2C1 on the self-link, served on the same core) | 230, three interrupts | 611, five | 1.00 through the pump (256 interrupts, 98 cycles a byte) and the engines (4) | 256 through the pump (117 cycles a byte), 3 through the engines | 1225 | - | 214 |
| STM32F446RE's FMPI2C1 (180 MHz; I2C1 on the self-link, served on the same core, its address-match stretch inside the figure) | 1131, two interrupts | 1680, four | 1.00 through the pump (256 interrupts, 128 cycles a byte) and the engines (2) | 256 through the pump (141 cycles a byte), 2 through the engines | 1168 | - | 509 |
| CH32V203C8T6 (120 MHz for the letter, the bus clock's 60 MHz ceiling; the CH32V303VCT6's `twi_peer`, its target polled by its own core) | 835, three interrupts | 1468, five | 1.00 through the pump (256 interrupts, 118 cycles a byte) and the engine (4, the core kept awake under a DMA block: 486 k busy against the pump's 51 k) | - | 838 | the EVT, against a target on the same core: 174 and 227 cycles a byte written and read, against 118 and 131 | 165 |
| CH32V303VCT6 (120 MHz for the letter, as the CH32V203; the CH32V203C8T6's `twi_peer`, polled by its own core) | 767, three interrupts | 1393, five | 1.00 through the pump (256 interrupts, 93 cycles a byte) and the engine (4, the core kept awake: 505 k busy against the pump's 44 k) | - | 746 | not run on this board | 183 |
| CH32V006K8U6 (48 MHz; the STM32G0's `twi_peer`) | 1373 (1896), four interrupts (five) | 1905 through the pump (2456) | 1.00 through the pump (264 interrupts) and the engine (9) | 262 through the pump, 9 through the engine | 1488 | WCH's polled master, its library having no interrupt-driven tenure: 766 and 907 fixed, the core held the whole tenure (3103 and 5581 busy) | 190 to 225 |
| STM32G0B1RE (64 MHz; I2C2 on the self-link; the 400 kHz rung 369.9 kHz on the desk's wire) | 1028 (1554), one interrupt (two) | 1651 (2377), two interrupts (four) | 1.00 through the pump (255 interrupts, 123 cycles a byte against the HAL's 168) and the engine (1) | 255 through the pump, 1 through the engine | 1189 | HAL v1.4.7: 1362 and 2289 fixed | 509, an engined write to an absent address |
| RP2040 (125 MHz; the WeAct RP2350B's `twi_peer` over the pair's wires) | 722, the same at 100 kHz and 1 MHz | 786 | 1.00, 31 | 32, 13 385 | 754 | the pico-sdk 2.3.1's polled loop: 398 fixed on a one-byte write, the core held the whole tenure (6528 busy) | 439 |
| RP2350, Cortex-M33 (150 MHz; the WeAct RP2040's `twi_peer`) | 392, the same at 100 kHz and 1 MHz | 441 | 1.00, 31 | 32, 9008 | 767 | the pico-sdk: 362 fixed (7714 busy) | 298 |
| RP2350, Hazard3 (150 MHz; the same) | 403 | 447 | 1.00, 31 | 32, 8417 | 385 | the pico-sdk: 354 fixed (7706 busy) | 279 |

What the rows say:

- The DW_apb_i2c host reads at half its receive FIFO and pours read
  commands as the bytes come back, so a tenure of eight bytes or less
  is one interrupt and a 255-byte read 32 where it was 285; every
  tenure ends on its own STOP, and the block's disable (a wait the bus
  timing sets, 1.5 us at 400 kHz and 6 at 100) is paid only when the
  address or the speed moves, which is what made the fixed cost the
  same at every speed ([the DesignWare I2C page](../dw_apb_i2c/README.md)).
  The far end of the RP rows is the desk partner's polled client, which
  gives a FIFO ahead and runs on the other chip, so nothing of it is in
  the figures.
- On the F1 lineage's event machine (the STM32F4, the CH32V203, the
  CH32V303, the CH32V006) one interrupt a byte is the silicon's, and
  the tenure's fixed cost is the engine's: the event interrupt raised
  by `start()` and dropped at the completion (the BTF a write's STOP
  leaves standing had re-entered the vector sixteen times a write at
  100 kHz on the F4), the first two data bytes loaded in the ADDR
  entry. The F429's one-byte write costs 139 cycles beyond the wire,
  the HAL's 387. The wait in `start()` for the last STOP stays (nothing
  in either chapter signals it leaving) and costs about one bit period
  when a tenure follows another at once. On the CH32V203 the pump's
  busy is 51 k for 255 bytes against the far board's target, the idle
  path taking one turn per wake, and the CH32V006's one-byte write
  1488 where it was 1759, the same cause.
- The AVR's TWI holds SCL from each flag until the software answers,
  so a long tenure at 400 kHz runs at the handler's pace: 1.12, the
  data sheet's own loop 1.11 ([the AVR TWI page](../avrxt/twi.md)).
- A block with a byte counter and AUTOEND but no FIFO (the STM32G0's)
  takes the pump's one interrupt a byte, and the tenure's fixed cost
  is what is left to cut: the first data byte preloaded into TXDR
  before the START, a read's last byte and a NACK both taken at STOPF,
  the enables written in one store each way; one interrupt for a
  one-byte write where there were two, a quarter under the HAL's
  ([the STM32G0 I2C page](../stm32g0/i2c.md), whose table is the same
  rung's).
- On the RP2350's Cortex-M33 a handler's cost from flash moves with
  where the linker puts it (188 to 365 cycles across three builds of
  one handler, up to six XIP misses a tenure); from SRAM 84. The
  default benches run both I2C vectors from the flash and their `_ram`
  images from SRAM, warm and cold side by side in the two families'
  platform documents ("A handler in SRAM").

## Letter j: the line writer, and letter l: the line without it

`util/line_writer.hpp`'s writer ([serial.md](serial.md), "LineWriter")
formats a print at the call into the caller's own buffer and hands the
line to the transport whole at its end, so its cost is one number: what
the line costs the object that prints it, there being no dispatch after
it. The STM32F446RE's preemptive-kernel suite measures it over a line
of 60 bytes in four texts and the end (the shape of the bench grammar's
verdict lines), printed from an object's dispatch four times in a row -
the first right after the console drained (cold), the second right
behind it (warm), the third with interrupts masked, the fourth in two
halves, the texts and the end - with the ruler (the DWT's CYCCNT, one
load), each figure the least and most of 16 over ten passes, the
ruler's own two reads taken off; the same line printed straight into
the transport, with interrupts masked, is measured in the same letter
as the comparison figure. Letter l of this instrument prints the same
line straight to the console in an image with no writer - the cost a
program under the cooperative kernel pays, where a line printed to the
transport is whole by construction - cold (right after the console
drained) and warm (right behind), and the transport's run alone; each
figure is the `wall` of the run with the least `wall` of 16, with its
`irq` and `isr` columns (the line's protocol, above), the ruler's own
read inside, and the least and most of the 16 beside it; three passes.
The ruler's read is a few cycles, so the two letters' figures compare
as they stand.

| (STM32F446RE, 180 MHz, 5 WS, ST's flash accelerator on; console 115200; ten passes of 16, three of letter l) | cycles |
|---|---|
| the line through the writer, from an object's dispatch, cold (one entry of the transmit handler inside, below) / warm, the line right behind the cold one (one entry inside) | 1989 to 2101 / 1901 to 2029 |
| the same, warm and masked (no entry inside) | 1774 to 1902 |
| of a warm one with interrupts enabled: the four texts into the stage / the line's end, its publish | 1101 to 1135 / 648 to 775 |
| `print()` straight into the transport, warm and masked, the same image | 1414 to 1530 |
| letter l: `print()` straight into the transport, no writer in the image, cold (`irq=2 isr=144`: two entries of the transmit handler, 144 cycles inside them) / warm (`irq=0`) | 1861 to 1981 / 1536 to 1652 |
| the transport's run alone, warm and masked: 2 bytes / 60 bytes (letter j) | 187 to 195 / 325 to 441 |
| letter l: the same, 2 bytes / 60 bytes | 181 to 193 / 345 to 425 |

Like against like: the writer's masked line against the direct print
masked in the same image, 1774 to 1902 against 1414 to 1530, some 360
cycles more, over a fifth; its warm line against letter l's warm direct
print, 1901 to 2029 against 1536 to 1652, some 370 more. What
the writer adds: the stage's resolution at each text, its take and the
lock, and every byte copied twice, into the stage and then into the
ring. The line's end is mostly the transport's run of 60 bytes, which
the runtime's `memcpy` copies in words whatever the ring's alignment
([runtime.md](runtime.md), "The copy across alignments"). The cold
line and the warm one behind it each carry one entry of the transport's
transmit handler, counted by letter j around each line (the most over
16 runs: 1, 1, 0, 0 for the cold, the warm, the masked and the split
line, in every pass; the kernel tick fell inside a line in four passes
of the ten, once in each). The mechanism, from `stm32f4/usart.hpp`'s transmit
path: a run that starts an idle transmitter pushes its first byte and
arms the transmit interrupt, which fires at once and hands the
transmitter that byte, inside the cold line; the transmitter takes the
byte into its shift register within one bit time, 8.7 us at 115200,
some 1560 cycles, and the interrupt fires again, inside the line that
follows, and hands it the second byte; the third comes a byte time
after that, 15600 cycles, past the four lines. So the warm line's
figure is the core's cycles and one handler entry, about 125 cycles
(1901 against 1774 at the least of one pass), and the masked line's is
the core's alone. Letter l's cold direct print counts two entries
(`irq=2`, 144 cycles inside them): its first run is the two-byte text at
the line's start, and both the first and the second interrupt fall
inside its 1861 cycles; its warm line, right behind, counts none.

## Not covered yet

Gaps, each with what would explain or measure it:

- The STM32F446RE transport's run costs some 180 cycles before its
  first byte, warm and masked (letters j and l: a run of 2 bytes 181 to
  195, of 60 bytes 325 to 441), against a listing of a call, the ring's
  indices, one push and two bit-band stores. Not split; what would: the
  DWT's LSU and fold counters around one run, and the run with its
  second store taken out.
- The direct line measures 1536 to 1652 in letter l and 1414 to 1530 in
  letter j, a gap of some 120 cycles. The ring's alignment at the line's
  longest run moves a line by some 30 cycles at most, that run being
  copied by one leaf at any alignment (60 bytes take 95 cycles aligned
  alike, 125 across); what else differs is known and not measured: the
  C-string verb is inlined in the suite's image and a call in the
  instrument's, and the code lies at other addresses, the accelerator's
  lines with it. What would: the line timed with the flash accelerator's
  caches reset before it in both images, the ring's head read before
  each line, and the code placed at one address in both.

- `bench_vx03`'s letter i run after its letter u in one session stalls
  at its first tenure, on the CH32V203C8 (whose two images keep the
  letters apart) and on the CH32V303VC, whose whole image passes
  letter i first in a fresh session and stalls on it after u. The
  stall is the SCL measurement's engined 255-byte read: I2C1 holds
  RxNE and BTF with DMAEN set, DMA1 channel 7 (I2C1's receive request,
  which USART2's transmit request shares) is enabled with 254 of its
  255 items left and its flags clear, and the event vector re-enters
  on BTF for ever. Clearing USART2's DMA request enables before its
  release does not cure it. Not explained; what would: the channel's
  and the USART's registers read at letter i's start in both orders,
  and letter u run with its transmit engine off channel 7.
- The cost of one code moves with its placement on the cores that
  fetch through a cache in front of slow flash: the STM32F446RE's idle
  turn (133 or 81 cycles) and its polled receive loop at 22.5 MHz
  (1.44 or 1.60) are the same instructions in two images, measured on
  one board; the F446's I2C handlers read 7 to 8 per cent fewer cycles
  a byte, the STM32G0's idle turn 12 more, with no instruction of
  theirs changed; and the F446's interrupt receiver reads 95 cycles an
  entry with the skipping ring, 98 with a plain Ring and 86 with a
  ring that marked its losses in place: three images on one board in
  one session whose receive paths differ in the ring alone. What would
  explain it: the DWT's fold, LSU and CPI counters around the loop in
  both images, and the loop's alignment to the ART's 128-bit lines.
- The RP2040's paced block runs 4 per cent long, its intervals 87 to
  628 cycles against 400, three of them missed by the poll. Not
  explained; what would: the request credits at the block's start and
  its first step read as the RP2350's letter reads them.
- The STM32G0's busy of a one-byte write is 979, 1189 and 833 cycles at
  the three rungs, its handlers and `start()` the same at each: the
  idle turns between the host's, the client's and the tick's entries,
  not split. What would: the stamps on each of the three vectors in
  one run.
- The instrument's two stated seams (a handler's entry and exit
  counted as idle, an interrupt between the idle call's return and the
  window's close counted twice) stay below its own cost in an idle
  turn, but not where a handler re-enters as soon as a window closes:
  on a pump faster than its handler the bench reads busy above wall
  (75 741 of 58 797 on the AVR at CLK_PER/4, 49 179 of 42 906 on the
  STM32F446RE at 22.5 MHz, 26 713 of 14 152 on the RP2350's M33 at
  37.5 MHz). Those lines' busy is not a cost; what would measure it:
  the window's close and the meter's question read under one mask.
