# TIM (CH32V00x)

The three timers of RM ch. 11, 12 and 13: TIM1, the advanced-control
timer; TIM2, the general-purpose one; and TIM3, a streamlined block
of the CH32V006/007 with no pad. Documents of record: the CH32V00X
reference manual V1.5 (11.2 and 11.3 for the structure and the modes,
11.4 for TIM1's registers, 12.2.2 for what TIM2 lacks, 12.3.9 and
12.4.17 for TIM2's dead-time pairs, 13.2 and 13.4 for TIM3, table 8-2
for the DMA channels), the CH32V006 datasheet V2.0 (table 2-1-1 for
the default pads).

## What the silicon does

- **TIM1 and TIM2 are the STM32F1's timers** under WCH's names at the
  F1's offsets: a 16-bit counter behind a 16-bit prescaler, four
  capture/compare channels, the slave controller with its trigger
  selection, the master TRGO, the external trigger with its filter and
  prescaler, sixteen bits at a four-byte stride but for the four
  CHxCVR, 32 bits wide so that bit 16 can carry a captured LEVEL.
- **What TIM1 has that TIM2 has not** (12.2.2): the repetition counter,
  three complementary outputs under the break/dead-time unit (BDTR),
  the break input and the idle states.
- **What TIM2 has instead**: DEAD TIME WITHOUT A BREAK UNIT. TIM2_DTCR
  pairs channel 3 with channel 1 and channel 4 with channel 2 as
  complementary outputs, each pair with its own dead time of 1..16
  module clocks and two polarities (12.3.9): a TIM2 pair costs two
  channels where a TIM1 pair costs one and its OCxN.
- **Three bits the F1 has not**, at the top of CTLR1: CAPLVL (the
  captured level in bit 16 of CHxCVR under a dual-edge capture), CAPOV
  (a capture after an overflow reads 0xFFFF), OE_MODE (the output pads
  float, or hold, with CEN clear).
- **TIM3 is another block** (ch. 13): a 16-bit counter with no
  prescaler, four compare registers, no capture, no pad, no interrupt;
  its channel 1 and 2 matches are internal pulses (the ADC's
  triggers), its channel 3 and 4 matches DMA requests; it counts CK_INT
  or TIM1's trigger.
- **The status register is write-zero-to-clear** (INTFR's RW0): a
  plain store of the complement, and a flag arriving between the read
  and the store survives.
- **The prescaler and the auto-reload are shadowed**, PSC at the next
  update, ATRLR too under ARPE.
- **The internal triggers** (table 11-2): TIM1's ITR1 is TIM2's TRGO,
  TIM2's ITR0 is TIM1's; TIM3 takes TIM1 (table 13-1).
- **A centre-aligned period is 2 x ATRLR counts** (measured), an
  update at each end of the triangle - the F1 lineage's count, not
  2 x (ATRLR + 1).
- **BIF cannot be cleared while the break input stands** (measured):
  a break handler on a held level re-enters for ever unless it
  silences its own enable.
- **TIM3's DMA request is ONE-SHOT** (measured): the channel 3 or 4
  match serves exactly one transfer after a reset of the block and
  never another - not the compare rewritten, not CCxDE toggled, not
  CEN toggled, not the counter zeroed, not the register read; only the
  RCC reset pulse re-arms it. TIM2's same request paces a channel
  every period.
- **The default pads on the CH32V006**: TIM1 CH1..4 on PD2, PA1, PC3,
  PC4, CH1N..3N on PD0, PA2, PD1 (SWDIO's pad), BKIN on PC2, ETR on
  PC5; TIM2 CH1/ETR on PD4, CH2..4 on PD3, PC0, PD7. The remaps are
  AFIO's, not touched yet.

## Types and verbs

[brio/ch32v00x/tim.hpp](../../brio/ch32v00x/tim.hpp):

- `Tim<n>`, n = 1 or 2: `init()`/`release()`, `configure(TimConfig)`
  (the time base, the counting mode, the three WCH bits, RPTCR on
  TIM1; the counter left stopped, UG run, UIF cleared), `enable()`,
  `count()`, `period()`, `prescaler()`, the software events, the flags
  and `clear_flags()`, `interrupts(mask, on)` for DMAINTENR's
  interrupt AND DMA enables, `isr()` (the raised-and-enabled flags,
  cleared, for any of the instance's vectors), `output_channel(ch,
  TimChannelConfig)` (mode, preload, polarity, the complementary
  output through CCER on TIM1 and through DTCR on TIM2, the idle
  states), `capture_channel(ch, TimCaptureConfig)`, `compare()`/
  `set_compare()`, `capture_raw()`/`captured_level()`, `slave(
  TimSlaveConfig)` (refusing a gated mode on TI1's edge detector and
  an ITR nothing is wired to), `master(TimMasterMode)`,
  `external_trigger(TimEtrConfig)`, `break_dead_time(TimBreakDeadTime)`
  and `main_output()` (TIM1), `pair_dead_time(pair, TimPairDeadTime)`
  (TIM2). Every verb naming a feature the instance lacks refuses.
  Channels are 0-based.
- `Tim3`: `configure(Config)` (period, direction, alignment, ARPE,
  `clocked_by_tim1`), `enable()`, `count()`, `compare()`/
  `set_compare()`, `compare_preload()`, `dma_request(ch, on)` for
  channels 3 and 4 - with the one-shot finding above.
- `TimPad<pad>`: a pad handed to a channel (`claim()`, `claim_input(
  pull)`, `release()`); `tim_default_pads` names the datasheet's.
- The tasks: `TimPwm<T, ch, top>` and `TimPairPwm<T, ch, top>`
  (util/pwm_channel.hpp's PwmChannel, the pair's dead time in BDTR's
  code on TIM1 and in DTCR's ticks on TIM2), `TimPeriodMeter<T>` (PWM
  input mode on two channels), `TimIntervalMeter<T, ch>` (one channel,
  the difference in the counter's modulus), `TimEventCounter<T>` (a
  timer clocked by another's trigger), `TimGatedCounter<T>` (a timer
  gated by another's OCxREF), `TimPeriodicTick<T>`, `TimOnePulse<T,
  ch>`.
- The arithmetic: `tim_dead_time_ticks(code)`/`tim_dead_time_code(
  ticks)` for BDTR's four ranges, `tim_internal_trigger()`/
  `tim_trigger_index_for()` for table 11-2, `tim_clock_hz(clock)` (a
  static clock only: a DynamicClock is refused).

## How to use it

```cpp
using Pwm = brio::TimPwm<brio::Tim<1>, 0, 999>;      // TIM1 CH1 on PD2
using Pad = brio::TimPad<brio::tim_default_pads::tim1_ch1>;

Pad::claim();
Pwm::setup(47);          // 48 MHz / 48 / 1000 = 1 kHz
Pwm::duty(250);          // a quarter, glitch-free (the compare is preloaded)
```

A frequency measured with no wire: TIM1 publishing its update on
TRGO, TIM2 counting it.

```cpp
brio::Tim<1>::configure({.prescaler = 47, .period = 99});   // 10 kHz
brio::Tim<1>::master(brio::TimMasterMode::update);
brio::TimEventCounter<brio::Tim<2>>::setup(brio::TimTrigger::itr0);
brio::Tim<1>::enable(true);
// ... 100 ms later: Tim<2>::count() == 1000
```

The vectors: `tim1_up_handler`, `tim1_cc_handler`, `tim1_trg_com_handler`,
`tim1_brk_handler`, `tim2_handler`, each calling `Tim<n>::isr()`.

## Bench findings

The reference suite is `test_ch32_tim` (19 verdicts in `z`, two
letters on one jumper, one probe letter outside `z`) on the
CH32V006K8U6 at 48 MHz.

- **The time base is exact**: TIM2 at 1 MHz against the STK, 100001
  counts in 100001 us; a TimPeriodicTick at 1 kHz delivers 199..200
  updates in 200 ms; an edge-aligned period is ATRLR + 1 counts (1000
  updates in a 100 ms window at 10 kHz), the repetition counter
  divides them by RPTCR + 1.
- **A centre-aligned period is 2 x ATRLR**: 1010 updates in 100 ms
  with ATRLR 99 at 1 MHz, two updates a period of 198 counts.
- **One timer measures another with no wire**: TIM2 counts TIM1's
  update on ITR0 (1000 in 100 ms), TIM1 counts TIM2's on ITR1 (500),
  and TIM2 gated by TIM1's OC1REF at 25% counts a quarter of a 10 ms
  window (54398 for 54464 after one wrap).
- **TIM3** counts TIM1's trigger in external clock mode 1 (1000 in
  100 ms) and HCLK undivided on CK_INT (24086 counts in 500 us); its
  channel 3 match reached DMA channel 1 with the sink holding the
  source - ONCE, the one-shot finding above, probed nine ways.
- **The break, staged from the pad's own pull**: PC2 pulled down with
  BKP active-high leaves MOE set; pulled up, MOE drops by hardware and
  BIF rises - and stands, unclearable, while the input does; under
  AOE the outputs come back by themselves when the pad is pulled down
  again.

## Not covered yet

Driver gaps, each with its reason:

- The DMA burst (DMACFGR/DMAADR) and the timers' requests as engine
  sources: born with a stream that needs them (TIM2's request is
  proven to pace a channel every period; TIM3's serves once).
- The encoder modes as a task: the resource has the three SMS codes; a
  task needs an encoder on the desk.
- The COM event and the commutation preload (CCPC/CCUS): motor
  control's, no user.
- TIM3's matches as ADC triggers: the ADC chapter's.
- The pad remaps: AFIO's.

Implemented but not bench-verified, each with what would measure it:

- **The captures**: TimPeriodMeter at three duties, TimIntervalMeter,
  TimOnePulse's width - the suite's letters f and g on the jumper PD2
  to PD4.
- The complementary outputs and their dead time on both timers, the
  idle states, the polarities: two pads on a scope or a logic analyser.
- The external trigger (ETR) and the external clock mode 2: a signal
  on PC5.
- CAPLVL's level bit and CAPOV's overflow marker: a dual-edge capture
  on the jumper, a slow input for the overflow.
