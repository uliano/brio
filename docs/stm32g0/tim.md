# Timers (STM32G0)

> **PROVISIONAL.** The time base, the counting modes, the
> capture/compare channels in both faces, the slave controller and the
> master TRGO, the break/dead-time unit, the repetition counter and the
> input multiplexer are implemented and bench-verified; the tasks over
> them give `util/pwm_channel.hpp` and `util/meter_sampler.hpp` their
> third silicon. What is still missing is in "Not covered yet".

Documents of record: RM0444 Rev 6 - TIM1 ch. 21, TIM2/TIM3/TIM4 ch. 22,
TIM6/TIM7 ch. 23, TIM14 ch. 24, TIM15/TIM16/TIM17 ch. 25, the timer
clock 5.2.13, the interrupt table 12.3 (table 61); DS13560 Rev 5 table 7
(the timer feature comparison) and tables 13..24 (the AF numbers of the
channel pads); errata ES0548 Rev 3 items 2.7.1..2.7.3, read on the bench
chip's revision Z column. Driver: `stm32g0/tim.hpp`; the per-instance
presence and vector facts come from `stm32g0/device_tables.hpp`. Bench
suite: `test_stm32_tim` (11 letters, 105 verdicts, wireless). Family
fixture `test/family_stm32g0/tim.cpp` plus seven negatives under
`tools/check_stm32g0.sh`.

## What the silicon does

**There is ONE `TIM_TypeDef` and TEN DIFFERENT TIMERS.** The device
header declares every register - SMCR, BDTR, RCR, CCMR2, TISEL - as a
member of one struct, so `TIM14->SMCR` compiles and writes a hole in the
address map. Nothing in the header says which timer has what. So the
geometry is the DOCUMENTS' (DS13560 table 7 and each chapter's ".2 main
features"), stated in the driver as a table keyed by instance and
reached only for an instance the header says exists:

| | counter | channels | complementary | slave | master TRGO | BDTR/RCR | up/down + centre | TISEL |
|---|---|---|---|---|---|---|---|---|
| TIM1 | 16-bit | 4 | 3 | yes | yes | yes | yes | yes |
| TIM2 | **32-bit** | 4 | - | yes | yes | - | yes | yes |
| TIM3, TIM4 | 16-bit | 4 | - | yes | yes | - | yes | yes |
| TIM6, TIM7 | 16-bit | - | - | - | yes | - | - | - |
| TIM14 | 16-bit | 1 | - | - | - | - | - | yes |
| TIM15 | 16-bit | 2 | 1 | yes | yes | yes | - | yes |
| TIM16, TIM17 | 16-bit | 1 | 1 | - | - | yes | - | yes |

Which of them a part bonds IS the header's, three ways over (`TIMn_BASE`,
the RCC enable/reset masks, the IRQn enumerators): the G0B1/G0C1 has all
ten, the G071 class all but TIM4, the G031 class TIM1, TIM2, TIM3,
TIM14, TIM16 and TIM17. Every verb that names a feature an instance does
not have returns false and writes nothing; the tasks that need one
refuse at compile time.

**The status register is `rc_w0`, not write-one-to-clear.** A flag of
`TIMx_SR` is cleared by writing ZERO to it and a write of one has no
effect (21.4.5), so this driver clears with `SR = ~flags` - a plain
store, no read-modify-write, and a flag that arrives between the read
and the store SURVIVES. Both other brio targets clear their flags by
writing ONES (the AVR's INTFLAGS, the SAM's INTFLAG) and so does this
family's own EXTI, which makes this the one register in the stratum
where the reflex is wrong.

**The prescaler and the auto-reload are shadowed.** PSC is copied into
the working register at the next UPDATE event and never before (21.4.11);
ARR is too when `CR1.ARPE` is set and is taken at once when it is clear
(21.4.12). A configuration is therefore only in force after an update,
which is why `configure()` ends with `EGR.UG` - a software update that
loads both shadows - and then clears the UIF that update raised.

**A compare register is PRELOADED by this driver, and that is a
choice.** `CCyPE` is clear out of reset, so a write to CCRy acts at once
and can produce a runt pulse; every output channel `output_channel()`
configures sets it, because a `PwmChannel::duty()` that can glitch is
not one a generic actuator can use. 21.3.10's own PWM recipe asks for
the same. The unbuffered behaviour is still reachable
(`TimChannelConfig::preload = false`), and letter `k` uses it.

**MOE is clear out of reset.** On the four break-capable timers nothing
reaches a pad until `BDTR.MOE` is raised - the trap that makes a first
TIM1 PWM look dead. A break clears it in hardware; `BDTR.AOE` is what
puts it back at the next update.

**Shared vectors are the rule** (table 61): TIM3 shares its line with
TIM4 on the G0B1 class, TIM6 with the DAC and LPTIM1, TIM7 with LPTIM2,
TIM16 and TIM17 with the two FDCAN interrupt lines - and TIM1 alone has
TWO vectors, its capture/compare events reporting separately from its
update, trigger, commutation and break. A handler is therefore a
dispatcher: it calls one `isr()` body per owner and each answers only
for the flags its own DIER enabled.

**The timer clock is PCLK here.** TIMPCLK is PCLK when the APB prescaler
is 1 and twice PCLK otherwise (5.2.13); `stm32g0/clock.hpp`'s `Clock<>`
pins that prescaler at 1, so TIMPCLK == `pclk_hz` == `hz` and
`tim_clock_hz(clock)` is that one number. `Tim<n>::clock_ok()` asks the
silicon whether the assumption still holds.

**The input multiplexer is what makes a capture measurable with no pad.**
Each `TIMx_TISEL` selects the source of TIy: code 0 is always the
channel's own pad, and above that the choice is per timer - TIM16
reaches LSI, LSE and the RTC wake-up (25.6.18), TIM17 HSI48/256, HSE/32
and the MCOs, TIM14 the RTC clock, HSE/32 and the MCOs, TIM1/2/3/4 the
comparator outputs, and TIM15's TI2 the capture signals of TIM2 and
TIM3. The driver exposes the raw code and names no source: the
vocabulary of what code 1 means belongs to the peripheral that owns the
signal (the samc EVSYS ruling), and half of these have no driver in this
stratum yet.

**The pad map is the datasheet's and nothing checks it.** Which AF
number carries TIM2_CH1 on PA5 is DS13560 table 13 (AF2), and the device
header has no symbol for it - `stm32g0/port.md` states that once for the
whole stratum. A timer pad is therefore a `PinSel` the caller writes and
the bench is the only check there is; three of those claims are measured
in `test_stm32_tim` (PA5 = TIM2_CH1 AF2, PA6 = TIM3_CH1 AF1 and
TIM1_BKIN AF2, PA7/PA8 = TIM1_CH1N/CH1 AF2).

**Errata.** All three TIM items of ES0548 Rev 3 apply to revision Z.
2.7.1 (a one-pulse trigger lost in master-slave reset + trigger with MSM
set) has "keep MSM reset" as its own workaround, which is this driver's
default - `TimSlaveConfig::master_slave` is false, and a caller that
sets it is told what it costs. 2.7.2 (a consecutive compare event
missed) has no workaround and is STAGED at the bench, see below. 2.7.3
(output compare clear with an external counter reset) is reachable
through `TimChannelConfig::clear_on_ocref_clr`, and the obligation is
stated on the verb.

## Types and verbs

- `Tim<n>` - the resource. `init()` / `release()` / `bus_clock(on)` /
  `reset()` / `clock_ok()`; `configure(TimConfig)` and its `config_valid`
  twin, `enable(on)` / `enabled()`; `count()` / `set_count()` /
  `count_update_flag()`, `prescaler()` / `set_prescaler()`, `period()` /
  `set_period()` / `auto_reload_preload()`, `repetition()` /
  `set_repetition()`; `update()`, `capture_compare_event(ch)`,
  `trigger_event()`, `break_event()`; `flags()` / `flag(mask)` /
  `clear_flags(mask)` with `update_flag`, `trigger_flag`, `break_flag`,
  `break2_flag`, `commutation_flag`, `compare_flag(ch)`,
  `overcapture_flag(ch)`; `interrupts(mask, on)` with
  `update_interrupt`, `trigger_interrupt`, `break_interrupt`,
  `compare_interrupt(ch)` and the three `*_dma` twins; `isr()`;
  `compare(ch)` / `set_compare(ch, v)`, `output_channel(ch,
  TimChannelConfig)`, `capture_channel(ch, TimCaptureConfig)`,
  `channel_enable` / `channel_enabled` / `complementary_enable`;
  `slave(TimSlaveConfig)` / `slave_mode()` / `slave_trigger()`,
  `master(TimMasterMode)`; `break_dead_time(TimBreakDeadTime)`,
  `main_output(on)`; `input_select(ch, code)`. Constants: `counter_bits`,
  `max_period`, `channels`, `complementary_channels`, `has_slave_mode`,
  `has_master_mode`, `has_break`, `has_repetition`, `has_direction`,
  `has_center_aligned`, `has_tisel`, `has_dma_burst`,
  `has_external_trigger`, `has_split_vector`, `irq()`, `cc_irq()`.
- Vocabulary: `TimDirection`, `TimAlignment` (edge and the three centre
  codes), `TimClockDivision` (CKD, which is tDTS), `TimTrigger`
  (ITR0..3, the TI1 edge detector, TI1, TI2, ETR), `TimSlaveMode`,
  `TimMasterMode`, `TimOutputMode`, `TimChannelSelect`,
  `TimCapturePolarity`, `TimCapturePrescaler`; `TimConfig`,
  `TimChannelConfig`, `TimCaptureConfig`, `TimSlaveConfig`,
  `TimBreakDeadTime`.
- Free functions: `tim_dead_time_ticks(dtg)` and `tim_dead_time_code(ticks)`
  (21.4.18's four ranges, the code search always rounding UP),
  `tim_internal_trigger(n, itr)` / `tim_internal_trigger_is_oc1(n, itr)`
  / `tim_trigger_index_for(slave, master)` (tables 119, 123, 130, both
  directions), `tim_clock_hz(clock)`.
- `TimPad<PinSel>` - `claim(speed, open_drain)` / `claim_input(pull)` /
  `release()`.
- Tasks: `TimPwm<T, ch, top>` and `TimPairPwm<T, ch, top>` (both
  `PwmChannel`), `TimPeriodMeter<T>` (PWM input mode: period and width),
  `TimIntervalMeter<T, ch>` (one channel, `interval()` for a capture
  ISR), `TimEventCounter<T>` (external clock mode 1 on a trigger),
  `TimGatedCounter<T>` (gated mode), `TimPeriodicTick<T>`,
  `TimOnePulse<T, ch>`.

## How to use it

A PWM output on the Nucleo's LED:

```cpp
constexpr brio::PinSel led{'A', 5, brio::PinFunction::af2};   // TIM2_CH1
using LedOut = brio::TimPad<led>;
using LedPwm = brio::TimPwm<brio::Tim<2>, 0, 999>;            // max = 999

brio::Tim<2>::init();
LedOut::claim();
LedPwm::setup(63);          // 64 MHz / 64 / 1000 = 1 kHz
LedPwm::duty(250);          // a quarter
```

A complementary pair with dead time (the four break-capable timers):

```cpp
using Pair = brio::TimPairPwm<brio::Tim<1>, 0, 999>;
brio::Tim<1>::init();
brio::TimPad<brio::PinSel{'A', 8, brio::PinFunction::af2}>::claim();   // CH1
brio::TimPad<brio::PinSel{'A', 7, brio::PinFunction::af2}>::claim();   // CH1N
Pair::setup(3, brio::tim_dead_time_code(128));   // 128 tDTS = 2 us at 64 MHz
Pair::duty(500);
```

One timer measuring another, with no pad and no wire: the master
publishes on TRGO and the slave takes it as a clock or as a gate.

```cpp
brio::Tim<2>::master(brio::TimMasterMode::update);     // or oc1ref: the waveform
constexpr auto itr = static_cast<brio::TimTrigger>(brio::tim_trigger_index_for(3, 2));
brio::TimEventCounter<brio::Tim<3>>::setup(itr);       // counts TIM2's periods
brio::TimGatedCounter<brio::Tim<3>>::setup(itr);       // counts its high time
```

A capture meter feeding a `util/meter_sampler.hpp` latch, with LSI as the
signal and no pad at all:

```cpp
using Lsi = brio::TimIntervalMeter<brio::Tim<16>, 0>;
using Latch = brio::MeterLatch<uint32_t, brio::Stm32Platform, 0>;

brio::Tim<16>::init();
brio::Tim<16>::input_select(0, 1);        // TI1SEL = LSI (RM0444 25.6.18)
Lsi::setup(0);
brio::Tim<16>::interrupts(Lsi::capture_interrupt, true);

extern "C" void TIM16_FDCAN_IT0_IRQHandler() {
    if (brio::Tim<16>::isr() & Lsi::capture_flag) {
        if (auto d = Lsi::interval()) { Latch::store(*d); }
    }
}
```

## Bench findings

Everything below is `test_stm32_tim`, wireless: no wire on the board and
nothing written to flash. Four techniques carry it - a timer counting a
timer over an ITR link, a pad read while a peripheral drives it, a
capture fed from LSI through TISEL, and a capture pad walked by its own
pull.

**The time base is exact.** A free-running 32-bit TIM2 counted 640115
ticks in 640176 CPU cycles (10 ms) - tick for tick, and past sixteen
bits, which only TIM2 can do. PSC = 63 gave 10002 counts where 10002
were due, so the divisor is PSC + 1 exactly. An edge-aligned period
measured 1000.0 counter ticks for ARR = 999 (below).

**The shadow registers, both of them, caught in the act.** A new
prescaler written under a running counter changed NOTHING - 1002 counts
in the millisecond after the write, against 1 in the millisecond after
`EGR.UG` loaded the shadow. With ARPE set, ARR moved from 20000 to 100
left the counter free to walk past 101 until the update, after which it
topped out at exactly 100.

**`CR1.URS` keeps a SOFTWARE update out of the flag**: `EGR.UG` with URS
set raised no UIF, and with URS clear it did.

**PWM read back through its own pad.** LD4 on PA5 at 64 kHz, sampled
60000 times through GPIOA's IDR: duties asked 0 / 250 / 500 / 750 / 1000
per mille read back 0 / 250 / 499 / 750 / 1000. **A sampling loop must
not branch on what it reads** - the first version of the pair census did,
and reported a 50 % complementary pair as 566 and 383 per mille, because
a branch makes the loop's own duration depend on the sampled state and
the slower state is then counted fewer times. Shifted-and-added, the same
pair reads 469 and 467.

**An EXTI line sees a pad a PERIPHERAL is driving.** The EXTI campaign
proved a line watches a pad its own application drives; here line 5
counted 200 rising edges of TIM2's waveform in 200002 us - one every
1000 us, the frequency TIMPCLK / (PSC + 1) / (ARR + 1) predicts - with
PA5 in alternate-function mode. 7.3.1's live input buffer holds for AF
mode too.

**One timer counts another, exactly.** TIM2 publishing its update on
TRGO and TIM3 clocked from ITR1 counted 6400 updates in 6400156 CPU
cycles: **1000 cycles per update where ARR + 1 = 1000**, with no pad, no
wire and no interrupt.

**A duty cycle measured inside the chip.** With TRGO = OC1REF the
trigger IS the waveform, and TIM3 in gated mode counts only its high
time: duties asked 250 / 500 / 750 per mille came back 250 / 500 / 749 -
the same numbers the pad's own sampling gives, from a mechanism that
shares nothing with it. **OC1REF exists whether or not CCER lets it
reach a pad**, but the channel has to be a waveform generator first: a
frozen channel publishes a constant and the gate never opens (the first
version of the letter measured zero).

**A capture that needs no pad anywhere.** TIM16 with TI1SEL = LSI
captured sixteen consecutive intervals of 1964 ticks (1963..1966), which
puts **LSI at 32586 Hz** - inside DS13560 table 46's 29.5..34 kHz and
within 0.2 % of the 32536 Hz `test_stm32_reset` measured through the
watchdog, two instruments sharing no mechanism. The capture prescaler
multiplied the interval exactly (3931 / 7861 / 15719 ticks for /2, /4,
/8 against 3928 / 7856 / 15712 due), `CCyOF` marked a capture that
landed on an unread one, and an input filter of 15 (eight samples at
fDTS/32) left the interval unchanged at 1964 ticks - a filter delays
both edges alike.

**A polling capture loop must have the console DRAINED first.** The
first version of that letter reported means of 11514 and 6536 ticks
where 3928 and 1964 were due: a transmit interrupt walking through the
loop makes it miss an edge, and a missed edge reads as an interval that
is a MULTIPLE of the true one. The suite now drains the console before
every window and carries `CCyOF` out with the readings, so a run that
did miss says so instead of averaging the damage in.

**PWM input mode**, on a TIM3_CH1 pad walked by its own pull (a capture
channel does not drive its pad, and PUPDR still does): captured period
1012 ticks and width 305 ticks against 1010 us and 305 us on the cycle
stopwatch. **The counter is reset ON the rising edge and the capture is
taken AT it, so a period reads as its own tick count and not one less** -
the opposite of the samc TC's capture, which clears and latches together
and always reads one short.

**The complementary pair and its dead time.** 60000 paired samples of
ONE IDR read: CH1 high 469 per mille, CH1N 467, **both high ZERO times**,
neither high 3840 (64 per mille). The two duties sum to 936 per mille
where 936 is 1000 less twice a 2 us dead time in a 62.5 us period, which
puts the **measured dead time at 2000 ns against the 2000 ns DTG = 128
asks for**. The dead time is counted in tDTS - the UNDIVIDED timer clock
over CKD - so it does not move with the prescaler.

**MOE, the break and AOE.** Clearing MOE took both outputs away at once
(0 of 20000 samples high on either pad; with OSSR clear the pads go back
to the GPIO and rest on their pulls), and raising it handed them back.
`EGR.BG` raised BIF and cleared MOE with no pad and no BKE. **The break
INPUT on a pad works out of the box**: `TIM1_AF1` comes up with BKINE
set, so PA6 at AF2 is already wired into the break logic, and with BKP
clear (active low) a pull-down on it raised BIF and cleared MOE. With AOE
set the silicon raised MOE again by itself at the next update.

**The repetition counter** divided the update event by RCR + 1: 801
updates in 50 ms at RCR 0 against 201 at RCR 3.

**Two timers on one vector, and one timer on two.** TIM3 at 1 kHz and
TIM4 at 2 kHz on the shared line gave 100 and 200 updates in 100 ms,
each answered by its own ISR body and neither consuming the other's
flags. TIM1 at 1 kHz gave 50 calls on the BRK/UP/TRG/COM vector and 49
on the CC vector in 50 ms. **A timer flag stands with its interrupt
masked** and is readable by a poller - where this same family's EXTI
keeps no pending bit for a masked line at all (exti.md).

**`TIMx_SR` really is rc_w0.** `EGR.UG` plus `EGR.CC1G` left SR = 0x3,
and a store of `~UIF` left 0x2: one flag goes and its neighbour stays,
with no read-modify-write to lose an arrival.

**A CENTRE-ALIGNED PERIOD IS 2 x ARR.** Counting the waveform's own
rising edges gave 12800 periods in 12800151 cycles edge-aligned
(**1000.0 ticks where ARR + 1 = 1000**) and 6406 periods in 12800139
cycles centre-aligned (**1998.1 ticks where 2 x ARR = 1998 and
2 x (ARR + 1) = 2000**), so the measurement tells the two candidate
formulas apart and 21.3.3's "0 to ARR-1 up, ARR to 1 down" is right. The
three CMS codes differ only in WHEN the compare flag rises: 100 flags in
100 ms counting up, 200 counting both.

**`MeterSampler` in a real kernel, on a third architecture with not one
line of `util/` changed.** TIM16 capturing LSI through its interrupt for
one second: **32535 capture interrupts, 9 samples published, 9 received**,
values 1964..1966 ticks, the latch's `missed()` 32524 and one reading
left fresh. **published + missed + leftover = 32534 = the number of
stores** exactly - a latch discards, and says exactly how much. (The
stores are one fewer than the captures: the first edge has no previous
one to measure an interval from.)

**ES0548 2.7.2 did NOT reproduce.** Staged eight times with the
prescaler at its maximum so one counter tick is 1.024 ms: a match at
CNT = CCR = ARR, then CCR moved to 0 inside that tick so the wrap makes a
second match in the very next counter cycle. The first match fired eight
times, the counter wrapped eight times, and **the second match raised its
flag and toggled its output all eight times** - with a control (CCR = 0
set well in advance) proving the instrument sensitive on both halves.
The erratum stands unrefuted rather than disproved: its own description
is of a REPEATING pattern driven at the counter's rate, where this letter
drives it once per period from software. 2.7.1 and 2.7.3 are named with
their reasons and not staged - 2.7.1 needs a trigger placed exactly at
CNT = ARR of a cascaded master, 2.7.3 needs an ocref_clr source (ETR or a
comparator) this stratum has neither a wire nor a driver for.

## The DMA requests a timer publishes

`TIMx_DIER`'s `UDE`, `TDE` and `CCxDE` bits are what makes a timer ASSERT
a DMA request; the NUMBER a channel has to listen for is RM0444 table
55's, and `Tim<n>` publishes it - `dma_update_request()`,
`dma_compare_request(ch)`, `dma_trigger_request()`, with
`ccr_address(ch)` for the register a stream writes into or reads out of.
The numbers live here and not in `stm32g0/device_tables.hpp` because no
device header of this pack declares one (the `DMAMUX_REQ_*` spellings are
ST's HAL/LL), and because of the standing ruling this stratum keeps for
TISEL and for the EXTI's lines above 15: a fabric driver owns the fabric,
a peripheral owns its own vocabulary. **TIM14 has no DMA request of any
kind** (DS13560 table 7), so every verb answers `dma_request_none` for it
and `has_dma_request` is false.

Measured in `test_stm32_dma` (which is where the numbers and the
arithmetic are pinned): an eight-entry duty table played into TIM2's CCR1
on its own update request, at a 16 kHz PWM, reads 521..522 per mille off
LD4's pad against the table's mean of 525, with the CPU touching not one
compare value; and TIM16's capture of the LSI - reached through TISEL
with no pad - streamed out by a ping-pong engine, four blocks of 32 with
every consecutive pair 30 us apart. A timer update at 32 MHz is also what
that suite uses to over-request the DMA's arbiter.

### The DMA burst engine (`test_stm32_dma` letter `l`)

`TIMx_DCR` and `TIMx_DMAR` turn ONE request into a walk of consecutive
registers, all reached through the single address `DMAR`, so a DMA
channel whose peripheral address never moves rewrites a whole waveform
per period. `Tim<n>::dma_burst(TimBurstBase, length)` writes the pair -
**the argument is the LENGTH and `DBL` is the length minus one**, which
is the off-by-one this driver refuses to make a caller carry - and
`dmar_address()` is the one address a channel is pointed at.

**THE MAP HAS HOLES AND THE WALK DOES NOT SKIP THEM.** `DBA` is a WORD
OFFSET from `TIMx_CR1` (21.4.20's own example list), so "ARR then CCR1
then CCR2" is not a burst of three but a burst of FOUR: offset 12
between `ARR` and `CCR1` is the repetition counter, which TIM2 does not
implement. A table meaning those three registers carries a zero there.

Measured on TIM2 driving LD4, four rows of four words played in a
circle off the update request, each row's `CCR1` at exactly half its own
`ARR`: **the pad reads 499 per mille whatever the period is doing**, and
the three registers stop coherent (`CCR1` exactly `(ARR+1)/2`, `CCR2`
exactly `(ARR+1)/4`). The CONTROL is the same four periods played one
word at a time into `ARR` alone with `CCR1` left at 500, where the
time-weighted duty is `4 x 500 / 10000`: **199 per mille measured
against 200 predicted**. The only difference between the two legs is
`DCR`'s length field.

`Tim<14>` has no burst engine (DS13560 table 7's "DMA request
generation" column, which is what `tim_has_dma_burst()` reads), so every
verb refuses on it and `dmar_address()` is null.

## Not covered yet

Driver gaps: encoder and hall-sensor modes (`SMS` 1..3 are spelled and refused
nowhere, but no task builds them and no bench signal exists for one);
the EXTERNAL trigger half of the slave controller (`SMCR`'s
`ECE`/`ETP`/`ETPS`/`ETF` and `TIMx_AF1`'s `ETRSEL`), which needs a pad or
a comparator; the commutation event (`CR2.CCPC`/`CCUS`, `EGR.COMG`) and
the `CCR5`/`CCR6` combined-PWM channels of TIM1; `CR2.TI1S` (the XOR of
the three inputs); the break's comparator inputs (`TIMx_AF1`'s
`BKCMPnE`); `TIMx_OR1`; the TIM1/TIM15 kernel-clock choice of PLLQCLK
(RCC_CCIPR, 5.2.13) - this stratum's PLL drives the R output only;
`DBGMCU`'s freeze bits (40.9.2); and the LPTIMs and IRTIM, which are
their own chapters.

Implemented, not bench-verified: TIM4, TIM6, TIM7, TIM14, TIM15 and
TIM17 as instances (the suite exercises TIM1, TIM2, TIM3, TIM4's vector
and TIM16); the output modes above `pwm2` (retriggerable one-pulse,
combined and asymmetric PWM); `TimOnePulse` and `TimPeriodicTick` on
silicon; `TimSlaveConfig::master_slave` (MSM), whose default is
ES0548 2.7.1's own workaround; `BDTR.LOCK`, which is one-way and would
cost a peripheral reset to undo; `OSSR`/`OSSI` other than clear;
`CR1.UIFREMAP` and `count_update_flag()`; the capture polarity `both`;
and every `TISEL` code but TIM16's LSI.
