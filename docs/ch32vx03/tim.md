# The timers (CH32V203)

One advanced-control timer and three general-purpose ones, the STM32F1's
blocks under WCH's register names: a sixteen-bit counter with two shadow
registers, four capture/compare channels in two faces each, a slave
controller that takes another timer's trigger as a clock, a master
output that publishes one, and on TIM1 a repetition counter, three
complementary outputs and a break input. Documents of record: the
CH32F/V20x_V30x_V31x reference manual V2.3 (chapter 14 for the
advanced-control timer, chapter 15 for the general-purpose ones, 9.5.1
and table 9-2 for the vectors, 3.3.1 for what the counter counts,
10.2.11 and table 10-15..10-18 for the pads) and the CH32V203 datasheet
V2.8 (table 2-1 for which timers a part offers, 3.2's pin tables for
which of their pads a package brings out). Ours is the CH32V20x_D6
device class for every part up to the CH32V203C8 and CH32V20x_D8 for the
CH32V203RB. Driver:
[brio/ch32vx03/tim.hpp](../../brio/ch32vx03/tim.hpp). Reference suite:
`test_vx03_tim`.

## What the silicon does

### What a part has

- **One advanced-control timer, three general-purpose ones, and no
  basic timer at all.** The datasheet's table 2-1 gives every part of
  this series TIM1 and TIM2..TIM4, and the 128 KB part a fourth
  general-purpose timer, TIM5, which chapter 15's own opening note makes
  THIRTY-TWO BITS on the CH32V20x_D8 class. Chapter 16 (TIM6, TIM7)
  describes silicon no CH32V203 carries. The driver folds those facts
  out of `device::` and no file but `parts/` names a part.
- **The register file is the F1's under WCH's names**: CTLR1, CTLR2,
  SMCFGR, DMAINTENR, INTFR, SWEVGR, CHCTLR1/2, CCER, CNT, PSC, ATRLR,
  RPTCR, CH1CVR..CH4CVR, BDTR, DMACFGR, DMAADR - sixteen bits each on a
  four-byte stride, with the counter, the auto-reload and the four
  capture/compare registers declared thirty-two bits wide (which is what
  they are on the 32-bit TIM5).
- **What the sister family added to this block is NOT here.** The
  CH32V00x's CTLR1 carries OE_MODE, CAPOV and CAPLVL above CKD and its
  TIM2 has a dead-time register of its own; on this family CTLR1 ends at
  CKD and TIM2 is an ordinary F1 timer. A program ported from there that
  wrote bits 13..15 of CTLR1 would write nothing at all.
- **The dual-edge capture register TIMx_AUX belongs to another class.**
  14.3.11's note names the D8, D8C and D8W lot numbers; nothing here
  reaches it.

### The time base

- **The prescaler and the auto-reload are shadowed** (14.2.3). PSC is
  copied into the working register at the next UPDATE event and never
  before; ATRLR is too when CTLR1.ARPE is set, and is taken at once when
  it is clear. So `configure()` ends with a software update event and
  clears the UIF it raises, and a caller that changes a period on a
  running timer decides which of the two it wants. Both halves measured.
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
- **There is no both-edge capture on this family.** CCER carries CCyP
  for the capture polarity and nothing else: CCyNP is the COMPLEMENTARY
  OUTPUT's polarity on the advanced timer (14.4.9) and Reserved on the
  general-purpose ones (15.4.9). The later STM32 families' "11 = both
  edges" encoding does not exist, so a caller that wants both edges uses
  two channels on one input - which is PWM input mode.
- **The input path of a channel is live whatever CCyS says**, so a
  channel driving its own pad is captured by another channel of the same
  timer through the INDIRECT mapping. That is the wireless instrument of
  this chapter, and the reference suite's own measurement of a PWM.

### The pads

- **A pad is a REMAP COLUMN, not a per-pin selector.** AFIO_PCFR1 holds
  one field per timer and its value selects a whole column of tables
  10-15..10-18, so TIM1's nine signals move together and a
  general-purpose timer's five do ([pin.md](pin.md) for the mechanism).
  The driver takes both the code and the pads from
  [afio.hpp](../../brio/ch32vx03/afio.hpp), so a remap and a pad cannot
  disagree, and a column whose pads this package does not bond is
  refused there.
- **An output channel wants the alternate-function nibble; a capture
  wants a plain input.** On this family the peripheral's input IS the
  pad's own input buffer (10.2.4), which has a consequence the relatives
  do not share:
- **A PAD IN PLAIN OUTPUT MODE REACHES THE TIMER'S INPUT.** Sixteen
  edges written on a pad configured as a GPIO output, with no
  alternate-function nibble at all, are sixteen captures - measured. On
  the STM32F4 the same experiment gives zero, its alternate-function
  input multiplexer being opened by the mode register
  ([the F4's document](../stm32f4/tim.md)). So on this family the CPU
  can stimulate a capture, an external clock or a break input by driving
  the pad through the PORT, and that is what the reference suite does
  where it needs a waveform of its own making.
- **In an ENCODER MODE a channel's output stage no longer reaches its
  pad.** Measured with one variable: the same channel, the same pad, the
  same forced output mode and the same CCER drives the pin while SMS is
  zero and leaves it low while SMS names an encoder mode. The
  STM32F4's own wireless quadrature - the timer driving its two tracks
  through its channels' output stages - is therefore not available here,
  and the port drives them instead.

### The slave controller and the triggers

- **Every timer of this family has the whole slave controller**: the
  three encoder modes, reset, gated, trigger and external clock mode 1,
  the external trigger input with its own polarity, prescaler and
  filter, and external clock mode 2.
- **Which timer each ITRx is, is per instance** (tables 14-2 and 15-2).
  Three of the table's entries name timers no part of this series
  carries - TIM1's ITR0 and TIM3's ITR2 are TIM5's, TIM2's ITR1 and
  TIM4's ITR3 are TIM8's - so the driver folds the table through the
  presence of the master and answers zero for a link that does not
  exist. `tim_trigger_index_for(slave, master)` reads it the way a
  caller thinks: name the master, get the index.
- **TIM2's ITR1 cannot be moved on this device class.** AFIO's
  TIM2ITR1_RM would point it at the Ethernet's time stamp or the USB
  frame marker; the field is read-only at zero here, measured
  ([pin.md](pin.md)), so the table above is the only connection there
  is.
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
- **The advanced timer has FOUR vectors and none of them is shared.**
  Table 9-2 gives TIM1 the break, update, trigger/commutation and
  capture/compare lines at 40..43 and the general-purpose timers one
  line each at 44..46; the TIM8..TIM10 entries that share those lines on
  the bigger families sit past this table's end. `vector_flags(irq)` is
  what a handler passes the ISR body so that one line cannot consume
  another's event.
- **DMAINTENR's DMA enables sit where INTFR's overcapture flags do**
  (bits 9..12), so the ISR body masks the enable register to its
  interrupt bits before it ANDs: a channel with a DMA request armed
  would otherwise eat a CCyOF that belongs to whoever reads the capture.
- **A break clears MOE asynchronously**, and with BDTR.AOE set the
  outputs come back at the next update event - a break as a cycle rather
  than a latch. Both measured, the break raised by software (SWEVGR.BG)
  and by the pad.
- **BKE and BKP need one bus period before they read back** (14.4.18's
  own note).

## Types and verbs

### The resource

`Tim<n>` is one TIMx block, n = 1..4 (and 5 on the part that has it).
What the instance IS comes out as constants - `counter_bits`,
`max_period`, `channels`, `complementary_channels`, `has_break`,
`has_repetition`, `has_slave_mode`, `has_encoder`, `has_master_mode`,
`has_external_trigger`, `has_ti1_xor`, `has_dma`, `on_pb2`,
`has_split_vectors` - and every verb that names a feature the instance
has not got returns false and writes nothing.

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
  `chcvr_address(ch)` - the addresses a DMA channel is pointed at
  ([dma.md](dma.md), where they are driven).

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
here), `TimCapturePrescaler` and `TimBurstBase`. Free functions:
`tim_dead_time_ticks(dtg)` and `tim_dead_time_code(ticks)` (the four
ranges of 14.4.18, the search rounding UP),
`tim_internal_trigger(n, itr)`, `tim_trigger_index_for(n, master)` and
the pad lookups.

### The tasks

| task | what it is |
|---|---|
| `TimPwm<T, ch, top>` | one PWM output as a `PwmChannel` whose `max` is the period; the frequency is the timer's, the duty the channel's |
| `TimPairPwm<T, ch, top>` | a channel and its complement with the silicon's dead time between them - TIM1's channels 1..3 alone |
| `TimPeriodMeter<T>` | the period AND the high time of a signal on TI1, in PWM input mode: two channels and a slave reset, what a capture body hands a `MeterLatch` |
| `TimIntervalMeter<T, ch>` | the interval between consecutive edges on ONE channel of a free-running counter, by subtraction |
| `TimEventCounter<T>` | a timer whose clock is another timer's trigger - a frequency counted with no pad |
| `TimGatedCounter<T>` | a timer counting its own clock while the trigger is high - a duty cycle measured internally |
| `TimPeriodicTick<T>` | an update event every period, and an interrupt on it |
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

The advanced timer's four vectors, each answering for its own flags:

```cpp
extern "C" BRIO_CH32_INTERRUPT void tim1_cc_handler() {
    const uint16_t hit = brio::Tim<1>::isr(
        brio::Tim<1>::vector_flags(brio::Irq::tim1_cc));
    // ... hit carries the capture/compare flags alone
}
```

## Bench findings

`test_vx03_tim`, ten letters, **36 pass, 0 fail** on a CH32V203C8 at
144 MHz with NOTHING WIRED (the suite's optional jumper, PA6 to PA1, was
absent for this run and letter `j` says so).

**The time base against the core's counter.** A 1 MHz counter advanced
34464 counts in 100 ms where 100000 modulo its sixteen bits is 34464 -
exact; a 1 kHz update arrived fifty times in 50 ms; twenty periods of
200, 1000 and 5000 us measured 4000, 20000 and 100000 us on the STK,
each exact to its printed microsecond.

**Both shadow registers, caught in the act.** An auto-reload of 100
written with ARPE set reads back at once and leaves the counter at 1858
two microseconds later - past the new period, which is only in force at
the update. A prescaler written while running moved the counter 770
counts in the next five microseconds and 5 in the five after a software
update - the old rate, then the new one.

**A timer capturing its own output.** Channel 1 making a 1000 us PWM and
channel 2 reading the SAME input through the indirect mapping captured
high times of **100, 250, 500 and 750 us** for compares of 100, 250, 500
and 750 - to the microsecond, four for four. The same arrangement is
what found the preload trap: before a software update was added to the
setup, those four readings came back one case late.

**The dead band, measured as the gap it is.** TIM1 at half duty with DTG
0xFF and CKD at four: the gap between channel 1 falling and its
complement rising measures **27902 to 27909 ns across runs against the
28000 ns** the code asks for, the two outputs are high for **471 us
each** of a 1000 us period, and of 20000 samples of the two pads
**none** caught them both high. The
DTG ladder reads back through all four of its ranges (32, 158, 376 and
1008 tDTS ticks for 0x20, 0x8F, 0xCF and 0xFF).

**The meters.** Six rising edges 1000 us apart, made by the CPU through a
channel's output stage, read as intervals of **1001 us**, with four
readings overwritten in the latch before the loop took one - and a
second `take()` with no new capture answers nothing, which is
design/meters.md's discard-stale rule. PWM input mode over the TI1 XOR,
with the CPU making a 300-of-1000 us wave, measured **1001 us of period
and 301 us of high time**.

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
drives its pad while SMS is zero and leaves it low while SMS names an
encoder mode.

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
  IS the request, so the arrangement stays the caller's.
- **The 32-bit TIM5 is compiled and not driven.** It exists on the
  128 KB part alone, which no board here carries; the family fixture
  instantiates every verb of it and the bench has never seen one.
- **A capture on a real signal from outside the chip.** Every
  measurement above is of a waveform this chip made - by a channel, by
  the port, or by another timer. What an encoder, a tachometer or a
  motor's Hall sensors do to these tasks needs a wire and a source.
- **Table 9-2's commutation event as a three-phase motor's own.**
  `preload_channels()` and the commutation vector are implemented and
  exercised, but the arrangement they exist for - six outputs switched
  together on a Hall transition - needs three pads, a driver stage and a
  motor.

Implemented but not bench-verified:

- **Centre-aligned counting and its three compare-flag policies**: the
  configuration is written and read back, but what the reference suite
  measures is edge-aligned. A scope or a capture on a second timer over
  a wire would say when CCyIF lands in each of the three modes.
- **The channels' input filters and prescalers** (ICyF, ICyPSC): both
  fields are written and read back, and no measurement here needs a
  filter - the waveforms are clean because the chip makes them. A noisy
  input is what would prove the filter.
- **The external trigger input (ETR) and external clock mode 2**: the
  polarity, prescaler and filter are written and read back; the pad that
  carries ETR on TIM3 is PD2, which this package does not bond, and
  TIM1's is PA12, which the USB owns on this board. A part or a board
  that brings one out would measure it.
- **The eight-bit repetition counter**: written and read back on TIM1,
  but nothing here counts update events at a ratio - a measurement of
  one update per N periods would.
- **The TI1 XOR as a Hall interface**: used here as a wireless path from
  one channel's output to TI1, which is not what it is for. Three
  sensors on three pads are.
