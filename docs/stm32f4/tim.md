# Timers (STM32F4)

Documents of record: RM0090 Rev 22 - ch. 17 the advanced-control TIM1 and
TIM8, ch. 18 the general-purpose TIM2..TIM5, ch. 19 TIM9..TIM14, ch. 20 the
basic TIM6 and TIM7 - with their twins RM0390 Rev 6 (ch. 15..18) and
RM0383 Rev 4 (ch. 12..14, which has no TIM8 and no basic timer). The
alternate-function numbers are the datasheets': DS10314 Rev 8 table 9,
DS10693 Rev 11 table 11, DocID024030 Rev 10 table 12. Errata: ES0287 Rev 6
2.6, ES0206 Rev 24 2.7 and ES0298 Rev 8 2.6 carry the SAME four items, so
they are the family's and are quoted under "what the silicon does".
Driver: `stm32f4/tim.hpp`; the per-instance geometry and the vector names
come from `stm32f4/device_tables.hpp`, and RCC_DCKCFGR's TIMPRE from
`stm32f4/clock.hpp`. Bench suite: `test_stm32f4_tim`. Family fixture
`test/family_stm32f4/tim.cpp` plus nine negatives under `brio check
stm32f4`.

## What the silicon does

**One register block, fourteen different timers.** The device header
declares ONE `TIM_TypeDef` and every instance is that struct, so
`TIM11->SMCR` compiles and writes a hole in the address map. What a timer
actually IS is the reference manual's, keyed by instance:

| | counter | channels | complementary | slave | encoder | master TRGO | BDTR | RCR | ETR | DMA | option reg |
|---|---|---|---|---|---|---|---|---|---|---|---|
| TIM1, TIM8 | 16 | 4 | 3 | yes | yes | yes | yes | yes | yes | burst | - |
| TIM2, TIM5 | **32** | 4 | - | yes | yes | yes | - | - | yes | burst | yes |
| TIM3, TIM4 | 16 | 4 | - | yes | yes | yes | - | - | yes | burst | - |
| TIM9, TIM12 | 16 | 2 | - | yes | **no** | **no** | - | - | **no** | **none** | - |
| TIM10, TIM11, TIM13, TIM14 | 16 | 1 | - | **no** | no | no | - | - | no | none | TIM11 |
| TIM6, TIM7 | 16 | **0** | - | no | no | yes | - | - | no | request only | - |

Two of those columns are easy to get wrong. TIM9 and TIM12 HAVE a slave
controller and no quadrature interface: SMS codes 001..011 are Reserved
there (RM0383 14.4.2), and so is TS = 111, the external trigger. And
TIM9..TIM14 have **no CR2 at all** - their register map goes straight from
CR1 to SMCR - so they publish nothing on TRGO, which is why the entries
that reach them in another timer's trigger table are their channel OUTPUTS
and not a TRGO.

**Which instances exist is the header's**, three ways over (`TIMn_BASE`,
the RCC masks, the IRQn enumerators): the F42x/F43x, F446, F405 class,
F412, F413/F423 and F469/F479 carry all fourteen; the F401 and F411 have
TIM1..TIM5 and TIM9..TIM11; the F410 has TIM1, TIM5, TIM6, TIM9 and TIM11
alone. Every verb that names a register an instance does not implement
returns false and writes nothing, and the tasks static_assert instead.

**The counter's clock is not the bus clock.** RM0383 6.2 and RM0090 7.2:
with a timer's APB prescaler at 1 the timer counts HCLK; with the bus
DIVIDED it counts **twice** its APB clock. And on every part but the
F405/F407/F415/F417 a bit of RCC_DCKCFGR, TIMPRE, changes that rule to
"HCLK at a prescaler of 1 or 2, four times PCLK beyond". So a timer on a
divided APB1 runs FASTER than the bus that carries it, and a period
computed from `pclk1_hz` is wrong by a factor of two. `tim_clock_hz()` and
`Tim<n>::clock_hz(clock)` do that arithmetic from the clock task's own
constexpr dividers; `clock_hz_now(hclk)` reads it back out of RCC_CFGR and
RCC_DCKCFGR, which is the witness a constexpr cannot be.

**The status register is rc_w0.** A flag of TIMx_SR is cleared by writing
ZERO to it and a write of one has no effect (17.4.5), so clearing is `SR =
~flags` - one store, no read-modify-write, and a flag that arrives between
a read and the store SURVIVES. Every other flag register in this stratum
is cleared by writing ones, so this is the one place the reflex is wrong.
Measured: a word of `0xFFFFFFFF` written over a standing UIF leaves it
exactly where it was.

**DIER's DMA enables overlap SR's overcapture flags.** CC1DE..CC4DE sit at
bits 9..12 and CC1OF..CC4OF sit at bits 9..12, so the `SR & DIER` an ISR
body is built on has to mask DIER to its low byte first - otherwise a
channel with a DMA request armed makes the handler swallow an overcapture
flag that belongs to whoever reads the capture. `interrupt_flags` is that
low byte and `isr()`'s default; the overcapture bits have no enable, no
vector and no place in any vector's set.

**The prescaler and the auto-reload are shadowed.** PSC is copied into the
working register at the next UPDATE event and never before (17.4.11); ARR
is too when CR1.ARPE is set. Both registers read back what was written all
the same - the shadow is not the register - so the only way to know a
configuration is in force is to have generated an update. `configure()`
therefore ends with EGR.UG and clears the UIF that raised. A compare
register is preloaded too, and **that is where a one-pulse timer bites**: it
sees no update between its configuration and its trigger, so the pulse
would be made against the ACTIVE compare's old value. `TimOnePulse::setup()`
generates the update that loads it (measured: without it the pulse comes
out the whole period wide).

**CCyS is writable only while the channel is OFF** (17.4.7: "CCyE = 0").
A channel switched from input to output with its enable still up keeps the
old direction and the write is dropped in silence - the timer goes on
capturing a pad the program believes it is driving. Both `capture_channel()`
and `output_channel()` clear the channel's CCER bits before they touch
CCMR, which costs an output a few cycles of release at reconfiguration time
and is the price of the rule.

**The vectors: four lines for two timers, one line for everyone else.**
TIM1 and TIM8 split their events over FOUR NVIC lines - break, update,
trigger/commutation, capture/compare - and three of the four are SHARED
with a small general-purpose timer: TIM1_BRK_TIM9, TIM1_UP_TIM10,
TIM1_TRG_COM_TIM11 and the TIM8 twins with TIM12, TIM13 and TIM14. TIM6
shares with the DAC where the part has one (the F412 has TIM6 and no DAC,
and its header spells `TIM6_IRQn` for it). The reserve derives every name
from PRESENCE - `TIM1_UP_TIM10_IRQn` where the part has a TIM10 and
`TIM1_UP_IRQn` where it has not - and never from a device name.

**The internal triggers are the wireless instrument of this chapter.**
Every four-channel timer publishes TRGO and every slave-capable one listens
on ITR0..ITR3; RM0090 tables 94, 98 and 101 say which timer each ITRx IS,
and the three manuals' tables are ONE table whose only differences are the
three entries naming TIM8 - printed Reserved on the parts that have no
TIM8. `tim_internal_trigger()` is that table with the presence probe over
it, and `tim_trigger_index_for(slave, master)` reads it the way a caller
thinks: name the master, get the ITRx. A master on `update` and a slave in
external clock mode 1 count each other exactly; a master on `oc1ref` and a
slave in gated mode measure a duty cycle. The errata's own obligation
travels with this: **the slave's clock must already be running before the
master sends**, and must not be changed while it does.

**Three timers have an option register**, and it is the other wireless
instrument. TIM5_OR's TI4_RMP routes the LSI, the LSE or the RTC wake-up
into TIM5's channel 4; TIM11_OR's TI1_RMP routes HSE_RTC (the HSE divided
by RCC_CFGR's RTCPRE) into TIM11's channel 1; TIM2_OR's ITR1_RMP moves
TIM2's ITR1 off TIM8's TRGO onto the Ethernet PTP trigger or a USB
start-of-frame. RM0383 6.2.11 names the first two as the intended way to
weigh an oscillator against the core clock, and that is what the suite does
with them.

**A GPIO in output mode does NOT reach a timer's input** - measured, and
the opposite of what the EXTI does with the same pad (see
[exti.md](exti.md)). The alternate-function input multiplexer is opened by
MODER, not by the AF nibble alone: sixteen edges written on a pad in output
mode whose AFR already names the timer produced zero captures. What DOES
work is the pad in ALTERNATE mode with the timer's own output stage driving
it - the chapter's **forced output mode** (17.3.7), where OCyREF follows
CCMRx and not the counter. The program writes a level into OCyM, the pad
follows, and TI1/TI2 see it, because the timer's input path is live for a
channel whether CCyS calls it an input or not. That is how a capture and a
quadrature interface are exercised on a board with no wire.

**The errata, four items, the same on all three sheets.**

- *PWM re-enabled in automatic output enable mode despite of system break*
  (ES0287 2.6.1). With BDTR.AOE set the outputs come back at the next
  update, and a SYSTEM break - the clock security system's - is not
  latched, so it comes back too. The workaround is the item's own: leave
  AOE clear and use OCyCE for cycle-by-cycle control. `TimBreakDeadTime`
  defaults `automatic_output_enable` FALSE for that reason and the verb
  says so.
- *TRGO and TRGO2 trigger output failure* (2.6.2), which the sheet itself
  calls a documentation issue: the slave's clock must be enabled before the
  master's first pulse and must not change while triggers are arriving.
  Stated on `master()` and obeyed by every task here that cascades.
- *Consecutive compare event missed in specific conditions* (2.6.3): two
  matches in adjacent counter cycles lose the second, which in toggle mode
  costs the short pulse and in centre mode costs the interrupt. No
  workaround; not staged here.
- *Output compare clear not working with external counter reset* (2.6.4):
  OCyCE together with a slave reset, combined reset or combined gated mode
  leaves the output inactive one extra PWM cycle. The driver exposes both
  halves, so the combination is reachable and the obligation is stated on
  the verb.

## Types and verbs

The driver owns the BLOCK and its geometry; what a signal on an ITRx or an
option register's code MEANS belongs to whatever owns that signal, so the
option register's codes are named (`Tim5Input4`, `Tim11Input1`,
`Tim2Trigger1`) and the DMA stream that serves a request is the DMA
chapter's.

Configuration structs: `TimConfig` (prescaler, period, direction,
alignment, clock division, auto-reload preload, one pulse, update disable,
update on overflow only, repetition), `TimChannelConfig` (mode, compare,
preload, fast, clear on ocref_clr, the two polarities, the two enables, the
two idle levels), `TimCaptureConfig` (select, polarity, prescaler, filter,
enable), `TimSlaveConfig` (mode, trigger, master/slave), `TimEtrConfig`
(inverted, prescaler, filter, clock mode 2), `TimBreakDeadTime` (dead time,
MOE, AOE, break enable and polarity, the two off states, lock),
`TimEncoderConfig` (mode, filter, the two inversions).

Vocabulary: `TimDirection`, `TimAlignment`, `TimClockDivision`,
`TimTrigger` (itr0..itr3, ti1_edge, ti1, ti2, etr), `TimSlaveMode`
(disabled, encoder1..3, reset, gated, trigger, external_clock1 - THREE bits
here, so there is no combined reset+trigger mode), `TimMasterMode`,
`TimOutputMode` (frozen, active/inactive on match, toggle, force
inactive/active, pwm1, pwm2 - three bits, so no combined or asymmetric
PWM), `TimChannelSelect`, `TimCapturePolarity`, `TimCapturePrescaler`,
`TimBurstBase`.

- `Tim<n>` - the resource. What this instance IS: `counter_bits`,
  `max_period`, `channels`, `complementary_channels`, `has_slave_mode`,
  `has_encoder`, `has_master_mode`, `has_break`, `has_repetition`,
  `has_direction`, `has_center_aligned`, `has_clock_division`,
  `has_external_trigger`, `has_ti1_xor`, `has_dma_request`,
  `has_dma_burst`, `has_option_register`, `has_split_vectors`, `on_apb2`.
  The vectors: `irq()`, `cc_irq()`, `break_irq()`, `trigger_irq()`,
  `vector_flags(irq)`. The clock: `clock_hz(clock, timpre)` (constexpr),
  `clock_hz_now(hclk)`. Bring-up: `bus_clock(on)` / `bus_clock()`,
  `reset()`, `init()`, `release()`, `regs()`. The time base:
  `config_valid(cfg)` (constexpr), `configure(cfg)`, `enable(on)` /
  `enabled()`, `count()` / `set_count(v)`, `direction()`, `prescaler()` /
  `set_prescaler(v)`, `period()` / `set_period(v)`,
  `auto_reload_preload()`, `repetition()` / `set_repetition(v)`. Events:
  `update()`, `capture_compare_event(ch)`, `trigger_event()`,
  `commutation_event()`, `break_event()`. Flags: `update_flag`,
  `trigger_flag`, `break_flag`, `commutation_flag`, `compare_flag(ch)`,
  `overcapture_flag(ch)`, `interrupt_flags` (SR's low byte, the only
  flags an interrupt can be raised for), `all_flags` (those plus the four
  overcapture bits), `flags()`, `flag(mask)`, `clear_flags(mask)`. Interrupts and DMA requests: `update_interrupt`,
  `trigger_interrupt`, `break_interrupt`, `commutation_interrupt`,
  `compare_interrupt(ch)`, `update_dma`, `trigger_dma`, `commutation_dma`,
  `compare_dma(ch)`, `interrupts(mask, on)` / `interrupts()`, `isr(mask)`.
  Channels: `compare(ch)` / `set_compare(ch, v)`, `output_channel(ch,
  cfg)`, `capture_channel(ch, cfg)`, `output_mode(ch, mode)` /
  `output_mode(ch)`, `channel_enable(ch, on)` / `channel_enabled(ch)`,
  `complementary_enable(ch, on)` / `complementary_enabled(ch)`. The slave
  and the master: `slave(cfg)`, `slave_mode()`, `slave_trigger()`,
  `master(mode)` / `master()`, `ti1_xor(on)` / `ti1_xor()`,
  `preload_channels(on, on_trigger)`, `compare_dma_on_update(on)`. Break
  and dead time: `break_dead_time(cfg)`, `main_output(on)` /
  `main_output()`, `dead_time_ticks()`. The external trigger:
  `external_trigger(cfg)`, `external_clock_mode2()`. The option register:
  `option(code)` / `option()`, and the typed pair per instance -
  `trigger1_source` on TIM2, `input4_source` on TIM5, `input1_source` on
  TIM11, each static_asserting the instance. DMA: `dma_burst(base,
  length)`, `burst_base()`, `burst_length()`, `dma_burst_off()`,
  `dmar_address()`, `ccr_address(ch)`.
- `TimPad<sel>` - a pad handed to a channel: `claim(speed, open_drain)`,
  `claim_input(pull)`, `drive(level)` (GPIO output with the AF nibble
  kept - see the finding above for what it does NOT do), `set()` /
  `clear()` / `read()`, `release()`.
- Free functions: `tim_clock_hz(clock, on_apb2, timpre)`,
  `tim_dead_time_ticks(dtg)` / `tim_dead_time_code(ticks)` (the search
  always rounds UP), `tim_internal_trigger(n, itr)` /
  `tim_internal_trigger_is_oc(n, itr)` / `tim_trigger_index_for(n,
  master)`, `tim_etr_config_valid(cfg)`.
- In the reserve (`stm32f4/device_tables.hpp`): `tim_base`, `tim_present`,
  `tim_bus_clock`, `tim_counter_bits`, `tim_max_period`, `tim_channels`,
  `tim_complementary_channels`, `tim_has_slave_mode`, `tim_has_encoder`,
  `tim_has_master_mode`, `tim_has_break`, `tim_has_repetition`,
  `tim_has_direction`, `tim_has_center_aligned`, `tim_has_clock_division`,
  `tim_has_external_trigger`, `tim_has_ti1_xor`, `tim_has_dma_request`,
  `tim_has_dma_burst`, `tim_has_option_register`, `tim_option_pos`,
  `tim_irq`, `tim_cc_irq`, `tim_break_irq`, `tim_trigger_irq`,
  `tim_has_split_vectors`, `rcc_has_timpre`.

The tasks, the same nine every stratum with timers offers:

- `TimPwm<T, ch, top>` - one PWM output, a `PwmChannel`
  (util/pwm_channel.hpp) whose `max` is the period: `setup(prescaler,
  mode, active_low)`, `duty(v)` / `duty()`.
- `TimPairPwm<T, ch, top>` - a channel and its complement with the dead
  time between them, the advanced-control timers' alone; also a
  `PwmChannel`, plus `dead_time_ticks()`.
- `TimPeriodMeter<T>` - PWM input mode: the period and the high time of
  one input, two channels and a slave reset. `setup(prescaler, filter,
  invert)`, `period_ticks()` / `width_ticks()` and the four flag constants.
- `TimIntervalMeter<T, ch>` - the interval between consecutive edges on one
  channel, which is what a capture ISR hands a `MeterLatch`:
  `setup(prescaler, filter, polarity, divider)`, `interval()`
  (`std::optional`, empty on the first edge), `restart()`, the flags.
- `TimEventCounter<T>` - external clock mode 1 on an ITRx: `setup(trigger,
  period)`, `count()`, `restart()`.
- `TimGatedCounter<T>` - the counter running only while the trigger is
  high: the same surface.
- `TimPeriodicTick<T>` - an update event every `period + 1` ticks and an
  interrupt on it: `setup(prescaler, period, interrupt)`, `stop()`, `flag`.
- `TimOnePulse<T, ch>` - one pulse of a chosen width after a chosen delay:
  `setup(prescaler, delay, width, active_low)`, `arm(trigger)`, `fire()`,
  `busy()`.
- `TimEncoder<T>` - the quadrature interface: `setup(TimEncoderConfig,
  period)`, `count()` / `set_count(v)`, `reversing()`.

## How to use it

A PWM on a pad, at a stated frequency:

```cpp
constexpr brio::PinSel wave{'A', 6, brio::PinFunction::af2};   // TIM3_CH1
using Wave = brio::Tim<3>;
using Lamp = brio::TimPwm<Wave, 0, 999>;                       // 1000 steps

brio::TimPad<wave>::claim();
Wave::init();
Lamp::setup(Wave::clock_hz(clock) / 1000u / 10'000u - 1u);     // 10 kHz
Lamp::duty(250);                                               // a quarter
```

A complementary pair with a dead time asked for in nanoseconds:

```cpp
using Pair = brio::TimPairPwm<brio::Tim<1>, 0, 999>;
constexpr uint32_t tdts_hz = brio::Tim<1>::clock_hz(clock) / 4;   // CKD /4
constexpr uint8_t dtg = brio::tim_dead_time_code(tdts_hz / 1'000'000u);  // 1 us
brio::TimPad<adv>::claim();
brio::TimPad<advn>::claim();
brio::Tim<1>::init();
Pair::setup(prescaler, dtg, brio::TimClockDivision::div4);
Pair::duty(500);
```

One timer counting another, with no pad and no CPU in the path - name the
MASTER and let the table find the link:

```cpp
constexpr uint8_t link = brio::tim_trigger_index_for(3, 2);   // TIM3 <- TIM2
static_assert(link != 0xFF, "no internal trigger between these two");

brio::Tim<3>::init();                       // THE SLAVE FIRST: its clock must
brio::TimEventCounter<brio::Tim<3>>::setup( //  be running before the master
    static_cast<brio::TimTrigger>(link));   //  sends (the errata's own rule)
brio::Tim<2>::init();
brio::Tim<2>::configure({.prescaler = 0, .period = 999});
brio::Tim<2>::master(brio::TimMasterMode::update);
brio::Tim<2>::enable(true);
const uint32_t periods = brio::TimEventCounter<brio::Tim<3>>::count();
```

An oscillator weighed against the core clock with nothing attached:

```cpp
using Meter = brio::Tim<5>;                 // 32-bit, and TI4_RMP reaches the LSE
Meter::init();
Meter::input4_source(brio::Tim5Input4::lse);
using Crystal = brio::TimIntervalMeter<Meter, 3>;
Crystal::setup(0, 0, brio::TimCapturePolarity::rising,
               brio::TimCapturePrescaler::every8);
// in the capture handler, or a polling loop:
if (Meter::flag(Crystal::capture_flag)) {
    if (auto d = Crystal::interval()) { latch.store(*d); }   // 8 crystal periods
}
```

A pulse of a known width on a trigger, and the vector of a timer that has
four of them:

```cpp
using Pulse = brio::TimOnePulse<brio::Tim<3>, 0>;
Pulse::setup(prescaler, 200, 500);          // 500 ticks, 200 after the trigger
Pulse::arm(brio::TimTrigger::itr0);         // or Pulse::fire() by software

extern "C" void TIM1_BRK_TIM9_IRQHandler() {   // one line, two owners
    if (brio::Tim<1>::isr(brio::Tim<1>::vector_flags(TIM1_BRK_TIM9_IRQn))) { ... }
    if (brio::Tim<9>::isr()) { ... }
}
```

Driving a capture input on a board with no wire - the channel stays in
alternate function and the CPU writes its output stage:

```cpp
brio::TimPad<enc_a>::claim();
Quad::output_channel(0, {.mode = brio::TimOutputMode::force_inactive,
                         .preload = false});
Quad::output_mode(0, brio::TimOutputMode::force_active);   // the pad goes high
```

## Bench findings

`test_stm32f4_tim`, 14 letters in `z` plus `h` by name, **68 pass, 0 fail**
in `z` and 5 more in `h`, WIRELESS - on an STM32F411CE (DEV_ID 0x431,
REV_ID 0x1000) at 100 MHz from its 25 MHz crystal, with an LSE fitted. The
pads driven are PA6 (TIM3_CH1), PA8 and PB13 (TIM1_CH1 and CH1N) and
PB6/PB7 (TIM4_CH1 and CH2); the console (PA9/PA10) and the SWD pads are
avoided, and nothing is attached to any of them.

**The reset state.** Every timer's bus clock is CLOSED out of reset, and
behind a closed gate every register of the block - ARR included, which is
all ones once clocked - reads zero. `init()`'s reset pulse puts them back
to the register map's values.

**TIMxCLK measured against the core clock.** With PCLK1 at 50 MHz (the APB1
prescaler at 2) an APB1 timer counts **99 999 753 Hz** and an APB2 timer
**99 976 539 Hz** - both HCLK, and both what `tim_clock_hz()` derives; the
APB1 number is the doubling rule caught in the act, since twice a halved
PCLK1 is HCLK again. Setting TIMPRE changes nothing on this part, and the
rule says why: a prescaler of 1 or 2 is HCLK either way.

**The prescaler's shadow.** PSC written to 9999 under a running counter
left it counting at the old rate - 20 058 ticks in 200 us - and the next
update event dropped it to 2 in the same 200 us.

**One timer counting another, exactly.** A master publishing its RESET
event on TRGO and a slave in external clock mode 1 on the ITRx the table
names: 100 software update events, **100 counts**, not 99 and not 101 - and
zero on any other ITRx. Free-running, a 1000-tick master over 99 734 us
gave 9974 update events against 9973 expected.

**A duty cycle measured inside the chip.** A master on OC1REF and a slave
in gated mode: with a compare of 250 out of 1000 the gate stood open
2 499 500 of 9 998 379 counter ticks, **249 per mille**; at a compare of
zero it never opened and at a compare of 1000 it never closed.

**A PWM read back on its own pad.** 100 000 reads of PA6's IDR while
TIM3_CH1 drove it at 10 kHz: 0 high at a compare of 0, **50 093** at 500 and
99 441 at 999 (the last one per mille is the single tick a compare equal to
the period cannot cover). PWM mode 2 gave 50 289 on the same compare and an
active-low pad at a compare of 250 gave 75 113 - two independent inversions.

**The repetition counter.** 50 ms of a 1000-tick TIM1 at RCR 0, 1, 3 and 7:
**5000 / 2500 / 1250 / 625** update events, exactly the four halvings, once
the measuring window was aligned to a tick at both ends.

**The dead time in nanoseconds.** DTG 0xFF decodes to 1008 tDTS ticks;
at CKD /4 on a 100 MHz timer that is 40 320 ns, and the two pads say
**39 900 ns** with the polling loop's own 400 ns subtracted - one per cent.
200 000 reads of the pair caught them both high **zero** times and both low
2669 times, which is the dead time itself.

**The break.** A software break (EGR.BG) cleared MOE and, with OSSI set,
parked the pad at its idle level: 49 644 high samples before, **0** after.
Raising MOE again brought the waveform straight back. With AOE set MOE came
back on its own within four reads of the register - the automatic re-enable
the errata's first item is about.

**One-pulse mode.** Fire to rise 200 us against 200 asked, high for
**501 us** against 500, and CEN clear afterwards. The pulse is made against
the ACTIVE compare register, and that register is preloaded: without an
update event between the configuration and the trigger the pulse came out
701 us wide - the whole period. `TimOnePulse::setup()` generates it.

**The oscillators through TIM5's option register.** The 32768 Hz crystal
weighs **32 769.167 Hz, +35 ppm** against the core clock (2048 captures of
eight crystal periods, about half a second), and `TimIntervalMeter` on the
same source gives 32 769.152 Hz - the two agree to half a part per million,
and both agree with the RTC chapter's own +36 ppm ([rtc.md](rtc.md)),
measured by a completely different instrument. The LSI weighs
**32 119 to 32 136 Hz** run to run, about 2 per cent below nominal and
inside RM0383's 17..47 kHz window; the RTC chapter's 32 124 Hz sits in the
same band.

**HSE_RTC through TIM11's option register** (letter `h`, which is not part
of `z`: freeing RTCPRE means resetting the backup domain, and that costs
the calendar and the twenty backup registers). With RTCPRE at 25 the input
weighs **1 000 000 Hz exactly** on a 16-bit counter, so the HSE is
25.000 MHz - the whole clock tree checked against itself, the PLL's ratio
included, with no instrument.

**A GPIO output does not reach a timer's input.** Sixteen edges written on
PB6 in output mode, its AF nibble already naming TIM4: **0 captures**. The
EXTI sees the same pad ([exti.md](exti.md)); the timer's alternate-function
input multiplexer does not, so MODER and not the nibble is what opens it.

**The encoder, on pads the timer drives itself.** Both channels in
alternate function with their output stage in forced mode: ten quadrature
cycles written by the CPU are **40 counts** in encoder mode 3 and 20 in
modes 1 and 2, ten backwards undo them exactly, CR1.DIR follows, and with
ARR at 39 a full forty counts came back to 0 while four counts below zero
landed on 36.

**A timer capturing its own output.** Channel 1 driving a 1000 us PWM and
channel 2 reading the SAME input through the INDIRECT mapping - PWM input
mode's own pairing - captured high times of **300 / 700 / 120 us** for
compares of 300, 700 and 120, to the microsecond; with the slave controller
reinitializing the counter on TI1's rising edge the same capture read 300.

**The vectors.** 200 us of a 100-tick TIM1 delivered 202 update calls and
200 compare calls on two DIFFERENT lines; the break vector ran once with
`0x80` in its mask and the trigger/commutation vector twice, once per
event. On the line TIM1's break SHARES with TIM9, both bodies ran and each
served only its own flags. A flag whose interrupt is not enabled is left
standing for a poller, which is what makes every measurement above possible
with no handler at all.

## Not covered yet

Driver gaps:

- **The DMA half.** The request ENABLES (UDE, TDE, COMDE, CCyDE), the burst
  engine (DCR/DMAR) and the addresses a stream is pointed at
  (`ccr_address()`, `dmar_address()`) are all here, and their register
  behaviour is checked; what is NOT here is this chapter's slice of the
  request mapping (RM0090 tables 43 and 44, RM0390 28 and 29, RM0383 27
  and 28), the `tim_dma_placements()` that would say which stream and
  channel serves which timer request. The reason is that no task in this
  file names a DMA engine slot yet - a duty table played into CCRy or a
  capture run harvested out of it is a task that has to exist first, and
  the placement table is born with it.
- **TIM8's second break input, TRGO2 and the ADC trigger.** This family's
  advanced-control timers have one break input and one TRGO; BKIN2 and
  TRGO2 are the F7/G4 generation's and no header of this pack declares
  them. Nothing to build.
- **The break INPUT PIN.** BKE, BKP, the filter-less break of this
  generation and the polarity are written and read back, and the break
  itself is raised by software (EGR.BG). Arming the pad needs a wire that
  can pull it, or a comparator this stratum has no driver for.
- **The commutation machinery for a three-phase bridge.** CCPC, CCUS and
  the preloaded channel configuration are exposed and the commutation event
  reaches its vector; what a motor driver would do with them - the six-step
  sequence, the Hall interface on CR2.TI1S - is an application's and is
  born with its first user.
- **DBGMCU's freeze bits**, which stop a counter while the core is halted:
  another block's register (and OpenOCD's own configuration writes some of
  them behind a debug session), so it belongs to a DBGMCU chapter that does
  not exist here.
- **A `Ticker` on a timer.** The kernel timebase is SysTick's on this
  family ([platform.md](platform.md)); a timer-based one would be worth
  building only for a program that needs SysTick for something else, or a
  tickless one that needs a counter that survives a Stop - the power
  chapter's question.

Implemented, not bench-verified:

- **TIM8 and the basic timers TIM6/TIM7**, and TIM12/TIM13/TIM14 with them:
  the part on the bench has none of them. They compile on every header that
  declares them and their geometry, vectors and refusals are asserted in
  the family fixture; what would measure them is the suite run on an
  STM32F429 or STM32F446, whose pad map for the five pads it drives is the
  one thing that has to be written first.
- **The chapter on STM32F429 and STM32F446 silicon.** The driver compiles
  on all twenty-three headers and the suite builds for the F446; every
  number above is the STM32F411's. On a part whose APB1 is divided by four
  the doubling rule and TIMPRE both do something the F411 cannot show -
  90 MHz timers becoming 180 - and that is the first thing to measure there.
- **Centre-aligned counting and the three CMS modes**, down-counting, and
  the alignment's effect on when CCyIF is raised: written, refused on the
  instances that have no CMS, and not staged - a centre-aligned PWM's
  waveform is measurable exactly as the edge-aligned one is here, and the
  three modes differ only in the flag's timing, which wants a capture on
  another timer to pin down.
- **The external trigger input** (ETR, external clock mode 2, ETPS, ETF)
  and the ocref_clr path that shares it: every code is written and read
  back, but ETR is a PAD on this family with no internal source to select -
  there is no ETRSEL multiplexer here as there is on the STM32G0 - so it
  needs a wire from a waveform pad.
- **The DMA burst engine's arithmetic** (DBA as a word offset, DBL as a
  length minus one): the register is written and read back and the base
  offsets are named from the register map, but nothing WALKS them here -
  that wants a stream pointed at `dmar_address()` and a timer task that
  owns it, which is the driver gap above.
- **PWM input mode as `TimPeriodMeter`** - the two-input-channel form. Its
  two ingredients are measured separately here (the indirect mapping's
  capture, and the slave reset that gives them one origin), but the task
  itself needs BOTH channels as inputs, which leaves the pad undriven: a
  wire from another timer's output pad, or a signal generator, is what
  would measure it.
- **The one-pulse mode's TRIGGER half.** `fire()` is measured; `arm()` on
  an ITRx composes two verbs that are each measured on their own and is not
  staged as a pair.
- **CR2.TI1S**, the XOR of the first three channel inputs: written and read
  back, and exercising it wants three pads carrying three different
  waveforms.
- **The BDTR LOCK levels.** Written and read back at level 0 and 1; the
  levels are ONE-WAY until the next peripheral reset, so proving that a
  locked field refuses a write costs a `reset()` per level and wants a
  letter of its own.
