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
  console's rate: the per-byte path every family pays
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
| SAM C21J18A (48 MHz OSC48M, 2 WS) | 65 | 150 | 262 | 3.47 | 3.40 | 4453 (4097 SERCOM5 + ticks), 168 (111), 0.99 | 255 k, 0.53 % |
| STM32G0B1RE (64 MHz PLL, 2 WS, the prefetch on) | 62 | 118 | 218 | 3.69 | 3.35 | 4453 (4098 USART2 + 355 ticks), 109 (67), 1.00 | 209 k, 0.33 % |
| CH32V203C8T6 (144 MHz PLL, zero-wait window) | 42 | 102 | 121 | 1.54 | 2.82 | 4452 (4097 USART1 + 355 ticks), 109 (67), 1.00 | 240 k, 0.17 % |
| STM32F446RE (180 MHz PLL, 5 WS, ART on) | 1 | 27 | 133 | 2.15 | 2.79 | 4453 (4097 USART2 + 356 ticks), 71, 1.00 | 132 k, 0.07 % |
| RP2350, Cortex-M33 (150 MHz PLL, XIP) | 3 | 18 | 156 | 1.65 | 2.04 | 474 388 (116 a byte: the PL011's storm), 36 a handler, 0.99 | 154 k, 0.10 % |
| RP2350, Hazard3 (150 MHz PLL, XIP) | 3 | 22 | 70 | 1.39 | 2.03 | 391 786 (96 a byte), 33 a handler, 0.99 | 68 k, 0.045 % |

What the rows say, and the two platform columns:

- Every print is wire-bound (`x` = 1.00) at one interrupt per byte, as
  the retrospective review counted: the per-byte shape costs CPU, not
  time, at a console's rate. The SAM's and the G0's transports take one
  interrupt more than the bytes (an empty entry from an idle transmitter
  on the SERCOM, a wasted pass after the first byte on the USART). THE
  PL011 IS THE EXCEPTION, AND THE NUMBER OF THE ROUND: on the RP2350 a
  print of 4096 bytes takes 474 388 interrupts on the M33 and 391 786 on
  Hazard3 - 116 and 96 a byte - because every `write_byte` the full ring
  refuses still pends the line, and the handler finds the FIFO full and
  re-arms it; the handlers hold 32 per cent of the core on the M33 for a
  print the wire bounds. The review's "one interrupt per byte in front
  of a 32-deep FIFO" was the quiet case; this is the loud one.
- The runtime's copy and fill sit at 2.4 to 4.2 times the core's floor:
  avr-libc's byte loops on the AVR (7 and 5 cycles a byte against 3 and
  1), and on the M0+ parts the `-Os` shape of `rt/rt.cpp`'s loops - four
  loads and four stores with a spill where a load-multiple pair would
  halve the turn, and a fill loop with two branches a turn. Findings for
  the runtime, explained and not yet closed.
- The instrument is a tenth to a third of what it measures on these
  cores: a ruler read is the tick count composed with the counter's
  position, 60 to 80 cycles, and a stamp pair two of them. Every line
  carries raw numbers; the row's own `ruler` and `stamp` are what the
  reader subtracts.
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

## Not covered yet

Implemented, not bench-verified:

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
