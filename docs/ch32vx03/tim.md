# The timers (CH32V203 and CH32V303)

The STM32F1's timer blocks under WCH's register names, in the three
kinds the reference manual describes: ADVANCED-CONTROL timers (a
sixteen-bit counter with two shadow registers, four capture/compare
channels in two faces each, a slave controller that takes another
timer's trigger as a clock, a master output that publishes one, a
repetition counter, three complementary outputs and a break input),
GENERAL-PURPOSE ones (the same without the last four), and on the
larger CH32V303 two BASIC ones (a time base and nothing else).
Documents of record: the CH32F/V20x_V30x_V31x reference manual V2.3
(chapter 14 for the advanced-control timers, chapter 15 for the
general-purpose ones, chapter 16 for the basic ones, 9.5.1 and table
9-2 for the vectors, 3.3.1 for what the counter counts, 10.2.11 and
tables 10-15 to 10-22 for the pads, tables 11-2 to 11-6 for the DMA
requests), the CH32V203 datasheet V2.8 (table 2-1 for which timers a
part offers, 3.2's pin tables for which of their pads a package brings
out) and the CH32V303/305/307/317 datasheet V3.5 (table 2-1-1, 3.2's
pin tables and 3.3's table 3-4). Ours are the CH32V20x_D6 device class
for every part up to the CH32V203C8, CH32V20x_D8 for the CH32V203RB and
CH32V30x_D8 for the four CH32V303. Driver:
[brio/ch32vx03/tim.hpp](../../brio/ch32vx03/tim.hpp). Reference suite:
`test_vx03_tim`.

## What the silicon does

### What a part has

- **The datasheets' tables say which timers a part offers.** Every
  CH32V203 and the two 128 KB CH32V303 (CB, RB) have one advanced-control
  timer, TIM1, and three general-purpose ones, TIM2..TIM4; the
  CH32V203RB adds TIM5, which chapter 15's own opening note makes
  THIRTY-TWO BITS on the CH32V20x_D8 class; and the 256 KB CH32V303 (RC,
  VC) carry the whole chapter set - FOUR advanced-control timers (TIM1,
  TIM8, TIM9, TIM10), four general-purpose ones (TIM2..TIM5, TIM5
  SIXTEEN bits there, measured) and the two basic timers of chapter 16,
  TIM6 and TIM7, which no CH32V203 has. The driver folds those facts
  out of the part table's masks and no file but `parts/` names a part.
- **The register file is the F1's under WCH's names**: CTLR1, CTLR2,
  SMCFGR, DMAINTENR, INTFR, SWEVGR, CHCTLR1/2, CCER, CNT, PSC, ATRLR,
  RPTCR, CH1CVR..CH4CVR, BDTR, DMACFGR, DMAADR - sixteen bits each on a
  four-byte stride, with the counter, the auto-reload and the four
  capture/compare registers declared thirty-two bits wide (which is what
  they are on the CH32V203RB's TIM5; on the CH32V303VCT6's TIM5 a
  written 0x12345 reads back 0x2345 in all three, measured). The
  CH32V30x_D8 adds TIMx_AUX at 0x50 (below).
- **A basic timer is a time base and nothing else** (16.2.2): no
  channel, no slave controller, no external trigger, no down-counting,
  no burst engine - CTLR1, CTLR2's MMS, UIE/UDE, UIF, UG, CNT, PSC and
  ATRLR are its whole register file (16.4). Its TRGO goes to the DAC
  and, on this family, to TIM9's ITR2 and ITR3 (table 14-2), which is
  how it is measured with no pad at all.
- **What the sister family added to this block is NOT here.** The
  CH32V00x's CTLR1 carries OE_MODE, CAPOV and CAPLVL above CKD and its
  TIM2 has a dead-time register of its own; on this family CTLR1 ends at
  CKD and TIM2 is an ordinary F1 timer. A program ported from there that
  wrote bits 13..15 of CTLR1 would write nothing at all.
- **The dual-edge capture register TIMx_AUX is the CH32V30x_D8's - and
  a LOT's.** 14.3.11 and 15.3.9 give channels 2..4 of every advanced
  and general-purpose timer a mode that captures both edges of an input
  and holds the pulse width in one register (CCyS = 11 with CAP_ED_CHy
  set), "only available for lot numbers where the penultimate sixth bit
  is not zero" - a rule no register states and the part number does not
  carry. On the CH32V303VCT6 of the bench TIMx_AUX keeps no bit written
  into it, on TIM1 and TIM2 alike, read over the debug port and from the
  program: this die's lot has no such register.

### The time base

- **The prescaler and the auto-reload are shadowed** (14.2.3). PSC is
  copied into the working register at the next UPDATE event and never
  before; ATRLR is too when CTLR1.ARPE is set, and is taken at once when
  it is clear. So `configure()` ends with a software update event and
  clears the UIF it raises, and a caller that changes a period on a
  running timer decides which of the two it wants. Both halves measured,
  on both parts.
- **A null auto-reload stops the counter** (14.4.12), which is a stopped
  timer wearing the face of a running one: the driver refuses a period
  of zero rather than letting it happen.
- **The counter counts TIMxCLK, which on this family IS HCLK** - at
  every rate this stratum can produce. RM 3.3.1's rule is "the bus clock
  when that bus's prescaler is 1, twice it otherwise", and the only
  divided bus here is PB1 at exactly two (the stratum caps it at 72 MHz,
  [clock.md](clock.md)), so both products are HCLK. The driver derives
  the rate from the clock task's own `timclk1_hz`/`timclk2_hz` - it
  folds at compile time - and `clock_hz_now(hclk)` reads the same number
  back out of RCC as a witness.
- **The timers take a STATIC clock.** Every period, prescaler and
  capture a timer holds is in TIMxCLK cycles, and a `DynamicClock` would
  move all of them at once with no rebase able to promise the same
  periods; the driver's own static_assert says so, as on the two other
  F1-lineage strata.

### The channels

- **CCyS is writable only with the channel off** (14.4.7). Both
  configuring verbs therefore clear the channel's CCER bits BEFORE they
  write CHCTLRx: without that a channel that is currently an input stays
  one, the field being dropped in silence, and the timer goes on
  capturing a pad the caller believes it is driving.
- **A compare register is preloaded by default here**, which is a choice
  and not the silicon's: OCyPE is clear out of reset (a write acts at
  once and can produce a runt pulse), and every output channel this
  driver configures sets it, because a `PwmChannel::duty()` that can
  glitch is not one an actuator above can use.
- **THE PRELOAD HAS A TRAP AT SETUP TIME.** A compare written into a
  preloaded channel reaches the shadow register at the next update event
  and NOT before, so a timer configured and started in one breath makes
  its FIRST period with whatever the shadow held - zero after a reset,
  the previous value otherwise. Measured, twice: four duties captured
  one case late, and a gated counter reading one period short of the
  truth. What fixes it is one software update event after the channel is
  configured, which is what `TimOnePulse::setup()` does for its own
  pulse and what any program that wants its first period right must do
  for itself.
- **There is no both-edge code in CCER.** CCER carries CCyP for the
  capture polarity and nothing else: CCyNP is the COMPLEMENTARY
  OUTPUT's polarity on the advanced timers (14.4.9) and Reserved on the
  general-purpose ones (15.4.9). The later STM32 families' "11 = both
  edges" encoding does not exist, so a caller that wants both edges uses
  two channels on one input - which is PWM input mode - or, on a
  CH32V303 whose lot has it, the dual-edge register above, which is a
  different register and not a CCER code.
- **The input path of a channel is live whatever CCyS says**, so a
  channel driving its own pad is captured by another channel of the same
  timer through the INDIRECT mapping. That is the wireless instrument of
  this chapter, and the reference suite's own measurement of a PWM.

### The pads

- **A pad is a REMAP COLUMN, not a per-pin selector.** AFIO_PCFR1 and
  PCFR2 hold one field per timer and its value selects a whole column
  of tables 10-15 to 10-22, so an advanced timer's nine signals move
  together and a general-purpose timer's five do ([pin.md](pin.md) for
  the mechanism). The driver takes both the code and the pads from
  [afio.hpp](../../brio/ch32vx03/afio.hpp), so a remap and a pad cannot
  disagree, and a column whose pads this package does not bond is
  refused there. TIM8's two columns, TIM9's four and TIM10's four are
  the CH32V303RC's and VC's; TIM3's external trigger is PD2 in every
  column, TIM4's is PE0 in both of its own (the V3.5 datasheet's table
  3-4 - a pad only the LQFP100 bonds), and TIM5 has no column at all:
  its channels are PA0..PA3 and its one field moves channel 4 to the
  LSI.
- **An output channel wants the alternate-function nibble; a capture
  wants a plain input.** On this family the peripheral's input IS the
  pad's own input buffer (10.2.4), which has a consequence the relatives
  do not share:
- **A PAD IN PLAIN OUTPUT MODE REACHES THE TIMER'S INPUT.** Sixteen
  edges written on a pad configured as a GPIO output, with no
  alternate-function nibble at all, are sixteen captures - measured on
  both parts. On the STM32F4 the same experiment gives zero, its
  alternate-function input multiplexer being opened by the mode register
  ([the F4's document](../stm32f4/tim.md)). So on this family the CPU
  can stimulate a capture, an external clock or a break input by driving
  the pad through the PORT, and that is what the reference suite does
  where it needs a waveform of its own making.
- **In an ENCODER MODE a channel's output stage no longer reaches its
  pad.** Measured with one variable: the same channel, the same pad, the
  same forced output mode and the same CCER drives the pin while SMS is
  zero and leaves it where it rests while SMS names an encoder mode. The
  STM32F4's own wireless quadrature - the timer driving its two tracks
  through its channels' output stages - is therefore not available here,
  and the port drives them instead.

### The slave controller and the triggers

- **Every advanced and general-purpose timer has the whole slave
  controller**: the three encoder modes, reset, gated, trigger and
  external clock mode 1, the external trigger input with its own
  polarity, prescaler and filter, and external clock mode 2.
- **Which timer each ITRx is, is per instance** (tables 14-2 and 15-2):
  TIM1 listens to TIM5/2/3/4, TIM8 to TIM1/2/4/5, TIM9 to TIM10/5/6/7 -
  the one timer the two basic timers' TRGO reaches - TIM10 to
  TIM9/2/4/5, TIM2 to TIM1/8/3/4, TIM3 to TIM1/2/5/4, TIM4 to TIM1/2/3/8
  and TIM5 to TIM2/3/4/8. The driver folds the table through the
  presence of the master and answers zero for a link a part has not got
  - TIM5 and TIM8 on most CH32V203, TIM8 on the CH32V203RB and the two
  128 KB CH32V303. `tim_trigger_index_for(slave, master)` reads it the
  way a caller thinks: name the master, get the index. Every link into
  TIM5, TIM8, TIM9 and TIM10, and every link TIM8 and TIM5 master, is
  measured on the CH32V303VCT6.
- **TIM2's ITR1 is "TIM8/USB/ETH" in table 15-2, and AFIO's
  TIM2ITR1_RM is the one field that could move it.** On the CH32V20x_D6
  the field is read-only at zero, measured ([pin.md](pin.md)), so the
  table's TIM8 - which that class has not got - is the only connection
  there is. On the CH32V303VCT6 the field takes a write, and TIM2's ITR1
  counts TIM8's TRGO at either value of it, measured; what else its
  second value connects is not.
- **A slave must be running before its master sends.** A slave whose
  clock is not yet enabled does not see the trigger that arrives while
  it is not, and a cascade configured the other way round counts one
  event short.

### The flags, the vectors and the break

- **The flag register is rc_w0**: a flag of INTFR is cleared by writing
  ZERO to it and a write of one has no effect (14.4.5), so clearing is
  one plain store of the complement - no read-modify-write, and a flag
  that arrives between a read and the store survives. Every other flag
  register of this stratum is cleared by writing ones.
- **An advanced timer has FOUR vectors and none of them is shared.**
  Table 9-2 gives TIM1 the break, update, trigger/commutation and
  capture/compare lines at 40..43 on every class, and the CH32V30x_D8's
  own table gives TIM8 59..62, TIM9 90..93 and TIM10 94..97 - entries
  the CH32V203's classes give to other peripherals or do not have. The
  general-purpose timers have one line each (44..46; TIM5 at 65 on the
  CH32V203RB and 66 on the CH32V303) and the basic timers one each (70
  and 71). `vector_flags(irq)` is what a
  handler passes the ISR body so that one line cannot consume another's
  event; measured on TIM1 on both parts and on TIM8, TIM9 and TIM10.
- **DMAINTENR's DMA enables sit where INTFR's overcapture flags do**
  (bits 9..12), so the ISR body masks the enable register to its
  interrupt bits before it ANDs: a channel with a DMA request armed
  would otherwise eat a CCyOF that belongs to whoever reads the capture.
- **A break clears MOE asynchronously**, and with BDTR.AOE set the
  outputs come back at the next update event - a break as a cycle rather
  than a latch. Both measured, the break raised by software (SWEVGR.BG)
  and by the pad the port drives - TIM1's BKIN on both parts, and
  TIM8's on the CH32V303VCT6.
- **A BREAK INPUT HELD AT ITS ACTIVE LEVEL RAISES BIF AGAIN AS SOON AS
  IT IS CLEARED.** 14.4.5 says only that the flag is "set by hardware
  and cleared by software"; measured on the CH32V303VCT6's TIM8, a
  break vector whose body clears BIF and returns while BKIN still
  stands is taken again at once, and the core does nothing else until
  the pad falls. A program whose break input can stand masks the break
  interrupt in its own body until the input has returned; the suite's
  break vector does exactly that, and BIF clears once the pad has
  fallen.
- **BKE and BKP need one bus period before they read back** (14.4.18's
  own note).

### The DMA requests

- **Each timer's requests are a controller's fixed channels**, the
  channel being the request on this family ([dma.md](dma.md) drives
  the DMA1 ones, and the burst engine's registers). The CH32V303's own
  timers raise theirs on DMA2 (tables 11-3 and 11-4): TIM5's CH4 and
  TRIG on channel 1, CH3 and UP on 2, CH2 on 4, CH1 on 5; TIM6's UP on
  3 and TIM7's on 4; TIM8's CH3 and UP on 1, CH4, TRIG and COM on 2, CH1
  on 3, CH2 on 5; TIM9's UP on 6, CH1 on 7, CH4 on 8, CH2 on 9, TRIG and
  COM on 10, CH3 on 11; TIM10's CH4 on 6, TRIG and COM on 7, CH1 on 8,
  CH3 on 9, CH2 on 10, UP on 11. The CH32V203RB's TIM5 raises its own on
  that part's single controller (table 11-6). They are stated here as
  the manual's data; the enables (`*_dma`) are this driver's verbs and
  the channel is the DMA chapter's.

## Types and verbs

### The resource

`Tim<n>` is one TIMx block, n = 1..10 where the part has it (a timer
the part has not got is a compile error). What the instance IS comes
out as constants - `counter_bits`, `max_period`, `channels`,
`complementary_channels`, `is_basic`, `has_break`, `has_repetition`,
`has_slave_mode`, `has_encoder`, `has_master_mode`,
`has_external_trigger`, `has_ti1_xor`, `has_dma`, `has_dma_burst`,
`has_up_down`, `has_dual_edge_capture`, `has_remap`, `on_pb2`,
`has_split_vectors` - and every verb that names a feature the instance
has not got returns false and writes nothing, except a channel verb on
a basic timer, which does not compile: the registers it would write are
not there.

- **The block**: `init()` (gate on, then the reset pulse), `release()`,
  `bus_clock()`, `reset()`, `regs()`.
- **The pads**: `remap(code)` writes the column and `remap()` reads it
  back; `channel_pad(code, ch)`, `complementary_pad(code, ch)`,
  `etr_pad(code)` and `break_pad(code)` name the pads of a column, and
  `TimPad<pad>` claims one - `claim()` for an output,
  `claim_input(pull)` for a capture, `read()`, `release()`.
- **The time base**: `configure(TimConfig)` and its compile-time
  judgment `config_valid()`, `enable(bool)`, `count()`/`set_count()`,
  `period()`/`set_period()`, `prescaler()`/`set_prescaler()`,
  `repetition()`/`set_repetition()`, `direction()`,
  `auto_reload_preload()`.
- **The events by software**: `update()`, `capture_compare_event(ch)`,
  `trigger_event()`, `commutation_event()`, `break_event()`.
- **The channels**: `output_channel(ch, TimChannelConfig)`,
  `capture_channel(ch, TimCaptureConfig)`, `output_mode(ch, mode)` for
  the forced modes alone, `channel_enable`/`complementary_enable` and
  their readers, `compare(ch)` (which acknowledges a capture by reading
  it) and `set_compare(ch, v)`.
- **The dual-edge capture**: `dual_edge_capture(ch, polarity, filter,
  prescaler)`, which writes the channel's CAP_ED bit, reads it back and
  answers false having written nothing else where the die keeps no
  such bit; `dual_edge_capture_off(ch)` and `dual_edge(ch)`.
- **The slave and the master**: `slave(TimSlaveConfig)`, `slave_mode()`,
  `slave_trigger()`, `master(TimMasterMode)`, `ti1_xor(bool)`,
  `preload_channels(on, on_trigger)`, `compare_dma_on_update(bool)`,
  `external_trigger(TimEtrConfig)`, `external_clock_mode2()`.
- **The break and dead time**: `break_dead_time(TimBreakDeadTime)`,
  `main_output(bool)`, `dead_time_ticks()`.
- **The flags and the interrupts**: the flag and enable constants
  (`update_flag`, `trigger_flag`, `break_flag`, `commutation_flag`,
  `compare_flag(ch)`, `overcapture_flag(ch)`, `interrupt_flags`,
  `all_flags`, and the matching `*_interrupt` and `*_dma`),
  `flags()`/`flag(mask)`/`clear_flags(mask)`, `interrupts(mask, on)`,
  `vector_flags(irq)`, `irq()`/`cc_irq()`/`break_irq()`/`trigger_irq()`,
  and `isr(mask)` - the body an application binds to a vector, which
  serves only what is ENABLED and leaves a polled flag standing.
- **The DMA burst engine**: `dma_burst(base, length)`, `burst_base()`,
  `burst_length()`, `dma_burst_off()`, `dmaadr_address()` and
  `chcvr_address(ch)` - the addresses a DMA channel is pointed at.

The configuration structs are `TimConfig` (prescaler, period,
direction, alignment, clock division, the two update controls, the
auto-reload preload, one-pulse mode, the repetition count),
`TimChannelConfig` (mode, compare, preload, fast, clear-on-ETRF, the two
polarities, the two enables, the two idle levels), `TimCaptureConfig`
(select, polarity, prescaler, filter, enable), `TimSlaveConfig`,
`TimEtrConfig`, `TimBreakDeadTime` and `TimEncoderConfig`; the
vocabulary enums are `TimDirection`, `TimAlignment`,
`TimClockDivision`, `TimTrigger`, `TimSlaveMode`, `TimMasterMode`,
`TimOutputMode`, `TimChannelSelect`, `TimCapturePolarity` (two values
here), `TimCapturePrescaler` and `TimBurstBase`. Free functions: the
part questions `tim_present(n)`, `tim_advanced(n)`, `tim_general(n)`,
`tim_basic(n)`, `tim_counter_bits(n)` and their siblings;
`tim_dead_time_ticks(dtg)` and `tim_dead_time_code(ticks)` (the four
ranges of 14.4.18, the search rounding UP);
`tim_internal_trigger(n, itr)`, `tim_trigger_index_for(n, master)`; and
the pad lookups `tim_channel_pad(n, code, ch)`,
`tim_complementary_pad(n, code, ch)`, `tim_etr_pad(n, code)` and
`tim_break_pad(n, code)`.

### The tasks

| task | what it is |
|---|---|
| `TimPwm<T, ch, top>` | one PWM output as a `PwmChannel` whose `max` is the period; the frequency is the timer's, the duty the channel's |
| `TimPairPwm<T, ch, top>` | a channel and its complement with the silicon's dead time between them - an advanced timer's channels 1..3 alone |
| `TimPeriodMeter<T>` | the period AND the high time of a signal on TI1, in PWM input mode: two channels and a slave reset, what a capture body hands a `MeterLatch` |
| `TimIntervalMeter<T, ch>` | the interval between consecutive edges on ONE channel of a free-running counter, by subtraction |
| `TimEventCounter<T>` | a timer whose clock is another timer's trigger - a frequency counted with no pad |
| `TimGatedCounter<T>` | a timer counting its own clock while the trigger is high - a duty cycle measured internally |
| `TimPeriodicTick<T>` | an update event every period, and an interrupt on it - a basic timer's one task |
| `TimOnePulse<T, ch>` | one pulse of a chosen width after a chosen delay, on a trigger |
| `TimEncoder<T>` | the quadrature interface: two inputs, the counter following the shaft in both directions |

## How to use it

A PWM output, and a duty that cannot glitch:

```cpp
using Lamp = brio::TimPwm<brio::Tim<3>, 0, 1000>;
constexpr brio::Pad pad = brio::tim_channel_pad(3, 0, 0);   // PA6 in column 0
(void)brio::Tim<3>::init();
(void)brio::Tim<3>::remap(0);
brio::TimPad<pad>::claim();
(void)Lamp::setup(143);          // 144 MHz / 144 / 1001 = about 1 kHz
Lamp::duty(250);
```

The complementary pair, whose dead time is asked for in tDTS ticks:

```cpp
using Pair = brio::TimPairPwm<brio::Tim<1>, 0, 2000>;
brio::Tim<1>::init();
(void)Pair::setup(0, brio::tim_dead_time_code(1000), brio::TimClockDivision::div4);
Pair::duty(1000);                // half, less the dead band on each edge
```

A frequency measured with nothing attached - one timer clocking another
over an internal trigger:

```cpp
constexpr uint8_t itr = brio::tim_trigger_index_for(2, 3);   // TIM3 reaches TIM2 on ITR2
(void)brio::TimEventCounter<brio::Tim<2>>::setup(static_cast<brio::TimTrigger>(itr));
(void)brio::Tim<3>::master(brio::TimMasterMode::update);     // the master, started after
brio::Tim<3>::enable(true);
```

A basic timer as a tick, and its TRGO counted by the one timer it
reaches (the CH32V303RC and VC):

```cpp
using Tick = brio::TimPeriodicTick<brio::Tim<6>>;
(void)Tick::setup(143, 99);                                  // 100 us
(void)brio::Tim<6>::master(brio::TimMasterMode::update);
constexpr uint8_t itr = brio::tim_trigger_index_for(9, 6);   // ITR2
```

A capture handed to a meter latch, which is where a reading crosses out
of the interrupt:

```cpp
using Edge = brio::TimIntervalMeter<brio::Tim<3>, 1>;
using Latch = brio::MeterLatch<uint32_t, P, 0>;

extern "C" BRIO_CH32_INTERRUPT void tim3_handler() {
    if ((brio::Tim<3>::isr() & Edge::capture_flag) != 0u) {
        if (const auto d = Edge::interval()) { Latch::store(*d); }
    }
}
```

An advanced timer's four vectors, each answering for its own flags -
and a break vector that masks itself while its input stands:

```cpp
extern "C" BRIO_CH32_INTERRUPT void tim1_cc_handler() {
    const uint16_t hit = brio::Tim<1>::isr(
        brio::Tim<1>::vector_flags(brio::Irq::tim1_cc));
    // ... hit carries the capture/compare flags alone
}

extern "C" BRIO_CH32_INTERRUPT void tim8_brk_handler() {
    using T = brio::Tim<8>;
    if ((T::isr(T::vector_flags(brio::Irq::tim8_brk)) & T::break_flag) != 0u) {
        T::interrupts(T::break_interrupt, false);   // until BKIN has returned
    }
}
```

## Bench findings

`test_vx03_tim` measures on both parts at 144 MHz. On the CH32V303VCT6
all nineteen letters run, both of the suite's optional jumpers in place
(PA6 to PA1, PC6 to PB8): **51 pass, 0 fail**. On the CH32V203C8T6 the
ten letters that part has run with NOTHING WIRED: **36 pass, 0 fail**,
the PA6-PA1 jumper absent and the letter that tests for it saying so.
The numbers below are both parts' where one number is given, and named
where they differ.

**The time base against the core's counter.** A 1 MHz counter advanced
34464 counts in 100 ms where 100000 modulo its sixteen bits is 34464 -
exact; a 1 kHz update arrived fifty times in 50 ms; twenty periods of
200, 1000 and 5000 us measured 4000, 20000 and 100000 us on the STK,
each exact to its printed microsecond.

**Both shadow registers, caught in the act.** An auto-reload of 100
written with ARPE set reads back at once and leaves the counter at 1858
(CH32V203C8T6) and 1852 (CH32V303VCT6) two microseconds later - past
the new period, which is only in force at the update. A prescaler
written while running moved the counter 770 and 774 counts in the next
five microseconds and 5 in the five after a software update - the old
rate, then the new one.

**A timer capturing its own output.** Channel 1 making a 1000 us PWM and
channel 2 reading the SAME input through the indirect mapping captured
high times of **100, 250, 500 and 750 us** for compares of 100, 250, 500
and 750 - to the microsecond, four for four. The same arrangement is
what found the preload trap: before a software update was added to the
setup, those four readings came back one case late.

**A second timer measuring the first, over the PA6-PA1 jumper**
(CH32V303VCT6): TIM2 read TIM3's wave as a period of **999 us** and a
high time of **299 us** for 1000 and 300 asked.

**The dead band, measured as the gap it is.** TIM1 at half duty with DTG
0xFF and CKD at four: the gap between channel 1 falling and its
complement rising measures **27902 to 27909 ns across runs against the
28000 ns** the code asks for, the two outputs are high for **471 us
each** of a 1000 us period, and of 20000 samples of the two pads
**none** caught them both high. The DTG ladder reads back through all
four of its ranges (32, 158, 376 and 1008 tDTS ticks for 0x20, 0x8F,
0xCF and 0xFF). TIM8's pair on PC6 and PA7 (CH32V303VCT6), each read on
its own pad: the same ladder, a gap of **27812 ns** for the same 28000,
and none of 20000 samples both high.

**The meters.** Six rising edges 1000 us apart, made by the CPU through a
channel's output stage, read as intervals of **1001 us**, with four
readings overwritten in the latch before the loop took one - and a
second `take()` with no new capture answers nothing, which is
design/meters.md's discard-stale rule. PWM input mode over the TI1 XOR,
with the CPU making a 300-of-1000 us wave, measured **1001 us of period
and 301 us of high time** on the CH32V203C8T6 and **1000 and 301** on
the CH32V303VCT6.

**Two timers, no pad.** Twenty milliseconds of a 10 kHz update on TRGO
counted **201** on the slave in external clock mode 1; twenty periods of
a 25 % duty gated the slave for **5000 us** of the 20000 - exactly the
high time. In reset mode the slave's counter sat at **0** after 5 ms of
triggers 200 us apart; in trigger mode it was idle before the master and
had counted **3000 us** after 3 ms of it, CEN set by hardware; gated on
the master's enable signal it counted **2000 us** while the master ran
and **0** after it stopped.

**The tasks.** A 500 us periodic tick arrived 100 times in 50 ms. A one
pulse asked for 200 us of delay and 500 us of width rose **200 us** after
its trigger and lasted **501 us** on the pad, and the counter stopped
itself at the update that ended it. The encoder, its two tracks written
by the PORT, counted **40** for ten quadrature cycles forward and
**-40** for ten back, with CTLR1.DIR following.

**The break.** The software break cleared MOE and raised BIF; with AOE
set it cleared MOE and the outputs were back three periods later; and
the BKIN pad driven high by the port itself broke the outputs - which is
the same finding as the capture's, the alternate-function input being
the pad's own buffer.

**The vectors.** Two milliseconds of a 100 us TIM1 delivered **20 update
and 20 compare** calls on two different lines, the trigger vector ran
once and the break vector once, and each body's mask carried only the
flags its vector answers for (0x2 and 0x80). TIM3's single line ran 20
times for ten periods of two events each. A flag whose interrupt is not
enabled stands after `isr()`, which is what every polled measurement in
the suite rests on.

**A plain output pad reaches a capture input, and a forced output does
not reach a pad in encoder mode.** Sixteen edges written on PA6 in
output mode: **16 captures**. The same channel's forced output mode
drives its pad while SMS is zero and leaves it where it rests while SMS
names an encoder mode - low on the CH32V203C8T6's board, high on the
CH32V303VCT6's, whose PB6 carries an external pull-up.

**The CH32V303VCT6's own timers.**

- **TIM8 captured by TIM4 over the PC6-PB8 jumper**: at 1 kHz with high
  times of 100, 250, 500, 750 and 900 counts of 1000, then 125 of 500
  (2 kHz), 50 of 200 (5 kHz), 3600 of 14400 (10 kHz on the undivided
  clock) and 720 of 1440 (100 kHz), TIM4 read every period and every
  high time to the count, nine for nine.
- **TIM8's four vectors**: 20 update and 20 compare calls on two lines
  in 2 ms of a 100 us period; the trigger/commutation line ran once for
  a trigger and once for a commutation, its last mask 0x20; the break
  line ran once for a software break (mask 0x80) and once for a 2 us
  pulse on BKIN (PA6) driven by the port, MOE set before the pulse and
  cleared by it, BIF standing in the body and cleared once the pad had
  fallen.
- **TIM9 and TIM10 with no wire**: each kept the core's time (50
  updates of a 1 kHz period in 50 ms), each channel 3's PWM with a
  compare of 300 left through its pad (PA4 and PC3) and came back on
  channel 4 through the indirect mapping as **300 us**, and each ran 20
  update and 20 compare calls on its own two lines in 2 ms of a 100 us
  period.
- **TIM5's width**: 0x12345 written into the auto-reload read 0x2345,
  0x1FFF0 into the counter read 0xFFF0 and 0x54321 into CH1CVR read
  0x4321; run from 0xFFF0 for 10 us the counter read 0x5E0 - sixteen
  bits, as the driver states for this class.
- **The basic timers**: TIM6 and TIM7 each made twenty update periods
  of 100 us in **2000 us** on the core's counter, ran their own vector
  50 times in 5 ms, and their TRGO reached TIM9's ITR2 and ITR3 - **201**
  updates counted in 20 ms, every one - with a channel's TRGO code
  refused on both.
- **The class's internal triggers**: every link into TIM8 (from TIM1,
  TIM2, TIM4, TIM5), into TIM9 (TIM10, TIM5, TIM6, TIM7) and into TIM10
  (TIM9, TIM2, TIM4, TIM5), TIM8 mastering TIM4 and TIM5, and TIM5
  mastering TIM1, TIM3 and being mastered by TIM2, TIM3 and TIM4 -
  nineteen links, each counting **201** of the master's two hundred
  updates; and TIM2's ITR1 counted TIM8's **201** with TIM2ITR1_RM at
  0 and **201** with it at 1.
- **TIM3's external trigger on PD2**, the pad driven by the port:
  forty pulses counted **40** on rising edges, **40** inverted, **20**
  prescaled by two and **5** by eight in external clock mode 2, and
  **40** through external clock mode 1 on ETRF.
- **The dual-edge capture**: TIM2's TIMx_AUX read 0x0 after CAP_ED_CH2
  was written, CHCTLR1 0x0 before and after - the verb answered false
  and wrote nothing else.

## Not covered yet

Driver gaps, each with its reason:

- **A task of this chapter that owns a DMA channel.** The enables, the
  burst engine's registers and the two addresses a controller is
  pointed at are implemented, and the DMA chapter drives them on the
  silicon - an update request pouring compares into a channel lap
  after lap, another sampling a second timer's counter, and the burst
  engine walking four registers per request
  ([dma.md](dma.md)). What no task here does is OWN the channel: which
  one to spend is the program's choice on a family where the channel
  IS the request, so the arrangement stays the caller's. The CH32V303's
  DMA2 requests above are the manual's data until a DMA2 driver serves
  them.
- **A break vector's policy for a break that stands.** `isr()` clears
  BIF and returns, which on a break input held at its active level is a
  livelock (measured); masking the break interrupt until the input
  returns, or disarming BKE, is the program's choice, and no verb here
  makes it.
- **A capture on a real signal from outside the chip.** Every
  measurement above is of a waveform the chip made - by a channel, by
  the port, or by another timer. What an encoder, a tachometer or a
  motor's Hall sensors do to these tasks needs a wire and a source.
- **Table 9-2's commutation event as a three-phase motor's own.**
  `preload_channels()` and the commutation vector are implemented and
  exercised, but the arrangement they exist for - six outputs switched
  together on a Hall transition - needs three pads, a driver stage and a
  motor.
- **The pads of the CH32V303's extra advanced timers beyond their
  default columns**: TIM8's second column, TIM9's and TIM10's upper
  three, and every pad of the three but the ones the suite drives
  (TIM8's CH1, CH1N and BKIN, TIM9's and TIM10's channel 3). They are
  table data judged by the bonding; the board has no wire to their
  other pads, and their complementary outputs but TIM8's first go to
  pads nothing on it reads.

Implemented but not bench-verified:

- **The dual-edge capture on a die that has it.** The verb writes
  CAP_ED_CHy, reads it back and configures the channel only when the
  bit stayed; on the CH32V303VCT6 of the bench it did not. A CH32V303
  from a lot whose penultimate sixth digit is not zero, measuring a
  pulse on TIM2's channel 2 over the PA6-PA1 jumper against the
  two-channel method, would measure it - and say which width the
  register holds.
- **The 32-bit TIM5 of the CH32V203RB.** It exists on that part alone,
  which no board here carries; the family fixture instantiates every
  verb of it, and the CH32V303's sixteen-bit TIM5 measures the same
  block at the other width.
- **Centre-aligned counting and its three compare-flag policies**: the
  configuration is written and read back, but what the reference suite
  measures is edge-aligned. A scope or a capture on a second timer over
  a wire would say when CCyIF lands in each of the three modes.
- **The channels' input filters and prescalers** (ICyF, ICyPSC): both
  fields are written and read back, and no measurement here needs a
  filter - the waveforms are clean because the chip makes them. A noisy
  input is what would prove the filter.
- **The external trigger input on the CH32V203**: TIM3's is PD2, which
  the LQFP48 does not bond, and TIM1's is PA12, which the USB owns on
  that board; the CH32V303VCT6 measures the same block's input on PD2
  above. A CH32V203 package that bonds PD2 would measure it there.
- **The eight-bit repetition counter**: written and read back on TIM1,
  but nothing here counts update events at a ratio - a measurement of
  one update per N periods would.
- **The TI1 XOR as a Hall interface**: used here as a wireless path from
  one channel's output to TI1, which is not what it is for. Three
  sensors on three pads are.
