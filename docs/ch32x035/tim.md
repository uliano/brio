# TIM1 and TIM2 (CH32X035)

This document is for a programmer who needs a timer's output: a PWM, a
single pulse of a stated width, a waveform drawn as a list of edges. It
covers the two advanced-control timers of RM ch. 12, one block twice,
the STM32F1's advanced timer under WCH's register names, in the subset
a waveform engine needs: the time base, the four channels as outputs in
every compare mode with the complementary outputs behind MOE (the main
output enable, the one bit without which no output of the timer is
driven), one-pulse mode, a compare's and the update's DMA request, the
remap. The driver is the register block as functions, `Tim<n>`, and a
pad handed to a timer signal, `TimPad`; there is no task over them.

The smallest use, a forced level on a complementary output's pad, the
opening of the edge list under "How to use it":

```cpp
using T1 = brio::Tim<1>;
T1::init();
(void)T1::remap(1);                                    // C2N on PB0, C3N on PB1
(void)T1::output_channel(1, {.mode = brio::TimOutputMode::force_active, .preload = false,
                             .enable = false, .complementary_enable = true});
T1::main_output(true);
T1::enable(true);                                      // the counter runs: the level reaches the pad
(void)brio::TimPad<brio::afio_tim1_pads(1).ch2n>::claim();
```

`init()` opens the block's clock gate and pulses its reset line;
`remap(1)` selects the column that puts the complementary outputs on
PB0 and PB1; `output_channel(1, ...)` configures the chapter's channel
2 (the driver counts channels from 0) as an output with the
complementary half enabled; `main_output(true)` is MOE; `enable(true)`
starts the counter; and `TimPad<...>::claim()` hands the pad to the
timer, named from the same column table the remap wrote, so the code
and the pad are one statement. Three things decide correctness even
there. The order of the last two lines: on this die a timer that has
never run holds its outputs low whatever mode is written, and a stopped
one holds the level it had, so a pad claimed before the first
`enable(true)` is driven low until the counter runs ("What the silicon
does", below). The counter counts HCLK, with no bus prescaler in
between, so every period, prescaler and compare value is in cycles of
the clock type's rate, and the timers take a static clock only. And a
pad of the debug port (PC18, PC19, which some columns of both timers
reach) is refused by `claim()` while the probe owns the port.

The rest of this page, in order: the documents of record; what the
silicon does, with the chapter's offer item by item; the types and
verbs; how to use it, the edge list streamed by DMA among the uses; the
bench findings; what is not covered. TIM3 (chapter 13's general-purpose
block) is another block and has no driver here.

## Documents of record

The CH32X035 reference manual V1.8 (12.1 for what the block has, 12.2.3
for the shadowed prescaler, 12.3.3, 12.3.4, 12.3.6, 12.3.7 and 12.3.9
for the compare modes, the complementary outputs and one pulse, 12.4 for
the registers, tables 12-3 and 12-4 for the maps; 8.3.2.1 for TIM1_RM
and TIM2_RM; 9.2.3's figure 9-1 for the requests; table 7-1 for the
vectors). Driver: [brio/ch32x035/tim.hpp](../../brio/ch32x035/tim.hpp),
the columns in [brio/ch32x035/afio.hpp](../../brio/ch32x035/afio.hpp);
reference suite `test_x035_tim` (a suite's letter, below, is one command
key of its console and the group of verdicts it runs, `z` the key that
runs them all), and the engine across a wire in the CH32X035's
`usart_peer`.

## What the silicon does

- **One block twice**: TIM1 on the PB2 bus at 0x40012C00, TIM2 on the PB1
  bus at 0x40000000 (tables 12-3, 12-4) - both "advanced-control", both
  with four channels, three complementary outputs, the brake, the
  repetition counter and the slave controller; only CTLR1's CMP_BK (a
  comparator as the brake input) is TIM1's alone. Sixteen-bit registers
  at a four-byte stride, the four CHxCVR 32 bits wide (bit 16 the
  captured level under CAPLVL), and a SPEC register at 0x50 the F1 has
  not got (an "alternate enable" of channel pairs).
- **CK_INT is HCLK**: the series has no peripheral-bus prescaler, so a
  count is HCLK / (PSC + 1), 21 ns at PSC 0 and 48 MHz.
- **The prescaler and the auto-reload are shadowed** (12.2.3): PSC is
  taken at the next update event (measured: PSC written alone left the
  count at 48 cycles a count until UG), ATRLR too under ARPE.
- **A UG reinitializes a RUNNING counter only** (measured, both
  directions): running, it gives 0 counting up and ATRLR counting down,
  as 12.3.1 says; on a stopped counter it loads the shadows and leaves
  the count where it was.
- **UDIS makes a UG no update at all** (12.4.1): the shadows are not
  loaded, no UIF rises, an overflow updates nothing - until UDIS is
  cleared, when the next overflow raises UIF and takes the PSC written
  meanwhile (measured). URS set, a UG loads the shadows and raises no
  UIF; an overflow still does.
- **INTFR is write-zero-to-clear** (12.4.5's RW0): a store of ones keeps
  every flag (measured).
- **CTLR1 resets to 0x2000** (measured, both timers): bit 13, "Reserved"
  with a reset value of zero in 12.4.1, reads one, and is writable both
  ways. It is the CH32V00x's OE_MODE position, and it does here what that
  bit's name says for the state MOE leaves an output in: set, an output
  MOE disables floats; clear, it is driven at its idle level OISx / OISxN
  (measured, below). WCH's library keeps it as it reads
  (`TIM_TimeBaseInit` is a read-modify-write of CTLR1).
- **DMAADR reads CTLR1** out of reset: it is the DMA burst's window onto
  the register DBA names (12.4.19, 12.4.20), and DBA resets to zero.
- **The outputs hold while the counter is stopped** (measured): with CEN
  clear every output - a channel's and a complementary one - keeps
  driving the level it had when the counter stopped, through 500 us and
  through a forced level written meanwhile; the running counter then
  takes the mode written. A timer that never ran since its reset holds
  its outputs low: a pad pre-charged high read low once handed to a
  forced-active channel. Chapter 12 says nothing of either; bit 13 changes
  neither.
- **MOE, the gate** (12.4.18, measured): set, the enabled outputs carry
  OCxREF; cleared with OSSI at its reset value and bit 13 at its own, an
  output is let go - a level charged onto the pad stays there, either way.
  Bit 13 cleared, MOE clear drives the idle level OIS2N against a charge
  of the other level, both ways.
- **The complementary output alone is OCxREF** (measured): CCxNE with
  CCxE clear puts OCxREF itself, through CCxNP, on OCxN with no dead time
  - the F1's output-control table, which chapter 12 does not print. CCxE
  and CCxNE together make OCxN the complement of OCx (dead time zero).
- **A software compare event does nothing to an output channel**
  (measured): 12.4.6 says CCxG sets CCxIF and raises the channel's
  interrupt and DMA request; on an output channel it raised no CCxIF,
  counter stopped or running. A real match does both. UG, on the other
  hand, raises the update's DMA request under UDE (measured, one item a
  UG on both timers).
- **The DMA requests are figure 9-1's** (measured, every row): TIM1's
  CC1..CC4 on channels 2, 3, 6 and 4 and its update on 5; TIM2's CC1..CC4
  on 5, 7, 1 and 7 and its update on 2.
- **Four vectors a timer** (table 7-1): TIM1 at 34..37 (brake, update,
  trigger/commutation, capture/compare), TIM2 at 38 (update) and 51..53
  (capture/compare, trigger/commutation, brake).

### The offer, item by item

What the chapter gives that bears on a waveform's cost, and what the
driver does with each:

| item | used | how, or why not |
|---|---|---|
| the time base (PSC, ATRLR, RPTCR, CMS, DIR, CKD, ARPE, UDIS, URS) | yes | `configure()`, one store of CTLR1 assembled whole but for UDIS, PSC, ATRLR, RPTCR, the counter's start written (ATRLR for an edge-aligned down counter, 0 otherwise: a UG does not move a stopped counter), then UG to load the shadows, its UIF cleared, and UDIS stored after it (under UDIS a UG loads nothing); the counter left stopped; `direction()` reads DIR, the hardware's in a centre-aligned mode |
| output compare, all eight OCxM codes | yes | `output_channel()` whole, `output_mode()` the field alone - what a waveform moves between toggle and a forced level, one read-modify-write |
| the compare preload OCxPE | yes | on by default (a PWM that cannot glitch), off for a value a DMA request writes at each match (12.3.3's step 4) |
| one-pulse mode | yes | `TimConfig::one_pulse`; the update clears CEN (measured) |
| the complementary outputs and MOE | yes | `TimChannelConfig::complementary_enable`, `main_output()`; on the QFN20 TIM1's C2N and C3N under code 001 are the only timer outputs that reach PB0 and PB1 |
| a compare's DMA request (CCxDE), the update's (UDE) | yes | `interrupts(compare_dma(ch) / update_dma)`; the channel each lands on is `compare_dma_channel(ch)` / `update_dma_channel`, read from dma_engine.hpp's table |
| CCxG, the software compare event | declined | it raises nothing on an output channel (measured); a request comes from a match |
| the remap (TIM1_RM, TIM2_RM) | yes | `remap(code)`, the columns as data (`afio_tim1_pads`, `afio_tim2_pads`); a code is refused only beyond its field or where its column bonds no pad here; a debug-port pad is refused by `TimPad::claim()` while the port lives |
| CTLR1's bit 13 | kept | its reset value, which configure() keeps as it reads: the driver gives no verb to a bit the manual calls reserved |
| the input face (capture, filters, PWM input), the slave controller and TRGO, ETR, the encoder modes | not driven | born with their first user (the gap list) |
| the brake, the dead time, OSSI/OSSR, the idle levels, LOCK | not driven | a power stage's; born with it |
| the COM event, the DMA burst (DMACFGR/DMAADR), SPEC's alternate enable, CMP_BK | not driven | no user |

## Types and verbs

[brio/ch32x035/tim.hpp](../../brio/ch32x035/tim.hpp) carries the
CH32V00x's TIM surface for the subset above:

- `Tim<n>`, n = 1 or 2 (`Tim<3>` does not compile: chapter 13's block).
  `init()` opens the gate and pulses the reset line, `release()` the
  reverse; `remap(code)`, `remap()`, `column(code)`; `configure(TimConfig)`
  and the time base's single verbs (`enable`, `count`, `set_count`,
  `prescaler`, `set_prescaler`, `period`, `set_period`, `repetition`,
  `set_repetition`, `one_pulse`, `direction`); `update()`; `flags()`, `flag()`,
  `clear_flags()`, `interrupts(mask, on)` over `update_flag`,
  `compare_flag(ch)`, `update_interrupt`, `compare_interrupt(ch)`,
  `update_dma`, `compare_dma(ch)`; `isr()`, the body for any of the
  timer's vectors; `ccr_address(ch)` for a DMA engine; `compare`,
  `set_compare`, `output_channel(ch, TimChannelConfig)`, `output_mode`,
  `channel_enable`, `complementary_enable`, `main_output`. Channels are
  0-based; a channel the block has not got, or a complementary output on
  channel 4, is answered false with nothing written.
- `TimPad<pad>` hands a pad to a timer signal, the pad named from its
  column - `TimPad<afio_tim1_pads(1).ch2n>` - so the code and the pad are
  one statement. A pad the package does not bond, or a signal the column
  has not got, does not compile; a pad of the debug port (TIM1's ETR
  under codes 010 and 011, TIM2's CH1 and C1N under 101 and 11x) is
  refused at run time while the port is the probe's.
- `tim_clock_hz(clock)` is HCLK, and a static clock only: a DynamicClock
  does not compile there.

## How to use it

An edge list on C2N (PB0), streamed by DMA on CH2's own request - the
waveform the `usart_peer` draws a frame with:

```cpp
using T1 = brio::Tim<1>;
using Edges = brio::DmaTxEngine<T1::compare_dma_channel(1), uint32_t>;   // channel 3
T1::init();
(void)T1::remap(1);                                    // C2N on PB0, C3N on PB1
(void)T1::output_channel(1, {.mode = brio::TimOutputMode::force_active, .preload = false,
                             .enable = false, .complementary_enable = true});
T1::main_output(true);
T1::enable(true);                                      // the counter runs: the level reaches the pad
(void)brio::TimPad<brio::afio_tim1_pads(1).ch2n>::claim();
Edges::arm(T1::ccr_address(1), 0u, brio::DmaPriority::very_high);
// then, a list of absolute compare values: the counter stopped, the first
// value stored, the rest started on the engine, toggle mode, CC2DE, CEN
```

One pulse of `w` counts after `d`: `configure({.prescaler = 0, .period = d
+ w - 1, .one_pulse = true})`, `set_compare(ch, d)`, `output_mode(ch,
pwm1)`, `enable(true)` - the pad high, then low for `w` counts, then high
again with the counter stopped by the update.

## Bench findings

`test_x035_tim` on the CH32X035F8U6, 47 verdicts with no wire of its own:
CH3 on PB11, C2N and C3N on PB0 and PB1 - the desk's UART wires, the far
CH32V006 dark - and TIM2's C2N on PB12, each pad's level read off INDR,
each edge placed by a sampler reading port B into a buffer with
interrupts masked, a sample every 9.01 HCLK cycles; the intervals
between flags stamped on the STK's cycle count, good to the poll's
lateness, which letter `f` measures in its own image (a turn of 108 to
112 cycles, 204 to 232 with the tick's vector in it: 316 to 344).

- **The reset values** are tables 12-3 and 12-4's but for CTLR1, 0x2000
  on both timers, and DMAADR, which reads CTLR1 (above).
- **The shadow**: 47.87 to 48.10 HCLK cycles a count at PSC 47 over the
  runs (a 100 us window and its own reads), the same after PSC 0 was
  written with no update, 1.00 after UG.
- **The levels**: a never-run timer's C2N read low with the pad
  pre-charged high; on a running counter, forced active high and forced
  inactive low; stopped high, still high 500 us later and after a forced
  inactive written; CC2NP inverting; CH3 and C3N complementary at both
  forced levels.
- **MOE**: cleared, C2N kept a charge of either level; bit 13 cleared,
  the idle level OIS2N won against a charge of the other, both ways.
- **The toggle**: a free-running toggle at a period of 1000 counts put an
  edge every 1000 to 1001 cycles on the pad (twelve edges).
- **The edge list**: ten absolute compare values at PSC 2 (three cycles a
  count), the first stored and nine streamed by channel 3 on CH2's
  request, the intervals 600 to 6000 cycles - each on the pad within 6
  to 14 cycles of the counts asked over the runs, the sampler's grain 19;
  the same on C3N through channel 6.
- **The engine's edge**: lists of eight values at PSC 0 with every
  interval the same - every value taken down to intervals of 10 HCLK
  cycles, one value of eight at 8, 6 and 4. A DMA item costs six cycles
  on this controller ([dma.md](dma.md)) and the request rides the match:
  ten cycles is the shortest interval an edge list keeps.
- **One pulse**: 2398 cycles low for 2400 counts at PSC 0, to the
  sampler's grain; CEN clear after it and the pad back high.
- **The match modes**: active on match low then high, inactive on match
  high then low, frozen holding both levels, toggle low then high, PWM
  mode 2 low under the compare and high from it.
- **The requests**: every row of figure 9-1 for both timers moved three
  items for three matches (or updates), none before the enable; three UGs
  under UDE moved three items on each timer's update channel; CC2G on an
  output channel raised no CC2IF, counter stopped or running.
- **The repetition counter** (letter `f`, thirty-two runs): on both
  timers one update in RPTCR + 1 periods - edge-aligned at RPTCR 0, 1, 4
  and 255 (100 us to 25.6 ms), centre-aligned at 0, 1 and 3, where an
  overflow and an underflow count one each - every interval within -208
  to +248 cycles of the counts over 616 intervals. Centre-aligned at an
  even RPTCR the updates alternate between overflow and underflow; at an
  odd one every update falls on the underflow, the counter counting up
  after it.
- **Centre-aligned counting's flag policies**: with ATRLR 500 and a
  compare of 200 at 1 MHz, CC1IF 1000 us apart and only counting down
  under CMS 01, only counting up under 10; under 11 on both, the gaps
  alternating 600 and 400 us.
- **The down counter**: an underflow and a compare's CC1IF once a
  period of ATRLR + 1 counts, DIR reading down; `configure()` starts it
  at ATRLR and the first underflow comes 48160 to 48244 cycles after
  `enable()`, a whole period of 48000. Before the driver wrote the
  start, `configure()`'s UG on the stopped counter left it at 0 and the
  first underflow came after one count.
- **UDIS and URS**: `configure()` asked for UDIS counts at the
  prescaler asked (48.0 cycles a count at PSC 47) - with UDIS stored
  before its UG, as the driver once did, the counter ran at the reset's
  PSC 0; under UDIS no UIF over three overflows and a PSC written
  meanwhile not taken; UDIS cleared, UIF at the next overflow and the
  new PSC (24.0 cycles a count). URS set: a UG raised no UIF and its new
  PSC was taken; overflows still raised UIF a period apart.
- **TIM2's outputs on a pad** (letter `g`): C2N reaches PB12 under
  TIM2_RM 101 and under 110 - forced active high, forced inactive low,
  CC2NP inverting, a toggle at a period of 1000 putting an edge every
  1001 cycles on the pad. Both codes put channel 1's CH1 and C1N on the
  debug port's PC19 and PC18: the letter never enables channel 1 nor
  claims those pads, and the probe attached as before after every run.

## Not covered yet

Driver gaps, each with its reason:

- **The channels' input face** - capture with its filter and prescaler,
  CAPLVL and CAPOV, PWM input - **the slave controller, TRGO, the
  external trigger and the encoder modes**: born with their first user (a
  meter, a timer chained to another, an encoder); no program of this
  stratum measures a pad's timing yet.
- **The brake unit**: BKE and BKP on the BKIN pad, CMP_BK, the dead time
  (DTG), OSSI, OSSR, the idle levels OISx, AOE and LOCK - a power stage's,
  born with the first program that drives one.
- **The COM event and the capture/compare preload (CCPC, CCUS)**: motor
  control's, no user.
- **The DMA burst** (DMACFGR, DMAADR): no stream needs a register block
  per request.
- **SPEC's alternate enable** (PWM_EN, PWM_OCx): the register's two
  sentences do not say what it alternates; born with a user that needs it
  and a measurement of it.
- **The tasks** the CH32V00x's timers carry (`TimPwm`, the meters, the
  periodic tick, one pulse as a task): born with their first user; the
  resource's verbs serve the waveform engine directly.
- **TIM3** (RM ch. 13): another block, born with its first user.
- **Every package but the QFN20**: the columns are data for all seven
  parts and the family check compiles every pad each package bonds; the
  bench has a CH32X035F8U6 alone.

