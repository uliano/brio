# Low-power timers (STM32G0)

> **PROVISIONAL.** Chapter 26 is implemented whole - both instances,
> every register and every field, the four kernel clocks, both counter
> modes, the trigger and input multiplexers, the three waveforms, the
> glitch filters, encoder mode, the two counter resets and the wake
> line - and the tasks over it give `util/pwm_channel.hpp` its fourth
> silicon and `util/power.hpp` its third sleep site on this target.
> What is still missing is in "Not covered yet".

Documents of record: RM0444 Rev 6 - LPTIM ch. 26, the kernel-clock
multiplexer 5.4.21 (RCC_CCIPR), the bus enable and reset 5.4.19/5.4.11,
the low-power modes 5.3 and 26.5, the EXTI line map table 65, the
interrupt table 12.3 (table 61), the DMAMUX trigger inputs table 56;
DS13560 Rev 5 tables 13..24 (the AF numbers of the LPTIM pads); errata
ES0548 Rev 3 items 2.8.1 and 2.8.2, read on the bench chip's revision Z
column. Driver: `stm32g0/lptim.hpp`; the third sleep site over it is
`Stm32LptimTimedSleepSite` in `stm32g0/sleep.hpp` and is documented in
[pwr.md](pwr.md). The per-instance presence, vector, EXTI and DMAMUX
facts come from `stm32g0/device_tables.hpp`. Bench suite:
`test_stm32_lptim` (9 letters, 68 verdicts, wireless). Family fixture
`test/family_stm32g0/lptim.cpp` plus five negatives under
`tools/check_stm32g0.sh` (the third sleep site's own two live with
`test/family_stm32g0/sleep.cpp` - see [pwr.md](pwr.md)).

## What the silicon does

A 16-bit up-counter with a 3-bit power-of-two prescaler, one compare and
one auto-reload, whose point is that IT DOES NOT NEED THE REST OF THE
CHIP. Two things follow from that and shape everything else:

**The counter's clock is chosen twice.** `RCC_CCIPR.LPTIMnSEL` picks the
KERNEL clock - PCLK, LSI, HSI16 or LSE, the same four for both instances
on every part of this pack - and then `CFGR.CKSEL` decides whether the
counter counts that clock or counts EDGES ON ITS OWN INPUT 1 instead.
With CKSEL = 1 no oscillator has to be running at all (26.4.4), which is
the "pulse counter" of 26.1; with CKSEL = 0 and COUNTMODE = 1 the input
is SAMPLED by the kernel clock, which loses nothing at the start but
needs the kernel clock to be faster than the input and the prescaler at
/1 (26.4.12).

**The APB side and the kernel side are different clock domains, and the
enable rules are OPPOSITE on the two halves.** This is the chapter's
easiest mistake and every verb in the driver checks its own side:

| register | may be written | citation |
| --- | --- | --- |
| CFGR, CFGR2, IER | only while DISABLED | 26.7.4, 26.7.9, 26.7.3 |
| CMP, ARR | only while ENABLED | 26.7.6, 26.7.7 |
| CR's CNTSTRT/SNGSTRT/COUNTRST/RSTARE | only while ENABLED - a write when disabled is DISCARDED BY HARDWARE | 26.4.7, 26.7.5 |

and a write to CMP or ARR is not finished when the store returns:
26.4.11 forbids a second write "before respectively the ARROK flag or
the CMPOK flag be set" and promises "unpredictable results" otherwise.

**Two instances, one `LPTIM_TypeDef`, and a table.** Table 135 gives
encoder mode to LPTIM1 alone and figure 271's footnote gives LPTIM2 only
one input channel - but the device header declares `CFGR.ENC`,
`ISR.UP/DOWN` and `CFGR2.IN2SEL` once, for the struct both instances
share, so `LPTIM2->CFGR |= LPTIM_CFGR_ENC` compiles and writes a bit
26.7.4 marks Reserved there. What an instance IS therefore comes from
the manual, stated with its citation in `device_tables.hpp` - the
`tim.hpp` geometry precedent - and every verb naming a missing feature
refuses.

**The trigger multiplexer's fifth row is not the same signal on the two
instances**: table 138 gives LPTIM1 `COMP3_OUT` where table 139 gives
LPTIM2 `TAMP_TRG3`. The other seven rows agree. The input multiplexers
disagree more: LPTIM1's input 1 reaches only the pad and COMP1_OUT
(table 140), while LPTIM2's reaches the pad, COMP1_OUT, COMP2_OUT and
their OR (table 142).

**The wake path is an EXTI DIRECT line** - 29 for LPTIM1, 30 for LPTIM2
(table 65). A direct line has no edge selection and no pending bit of
its own (the peripheral's own ISR flag IS the pending state), but its
IMR bit must stand or the interrupt does not bring the core out of a
Stop.

**Two errata, and both are answered by construction.** ES0548 2.8.1:
clearing `CR.ENABLE` near an interrupt can freeze the wake-up signal in
its active state, after which the device cannot enter Stop at all; the
workaround is not a sequence but a SUBSTITUTION - "do not clear its
ENABLE bit... instead, reset the whole LPTIMx peripheral via the RCC
controller". So NO VERB IN THIS DRIVER WRITES ENABLE = 0: `disable()`
and `reset()` are both an `RCC_APBRSTR1` pulse, and the price is that a
disable forgets the configuration. ES0548 2.8.2: with at least one
interrupt enabled, clearing a flag whose interrupt is DISABLED at the
same instant as a new event is detected leaves the interrupt line
"permanently stuck high"; the workaround is that flags are cleared only
inside the interrupt routine, disabled-interrupt ones first.

## Types and verbs

`Lptim<n>` (n = 1, 2) is the resource - one register fact per verb, with
the ordering rules left to the tasks:

- `init()` / `release()` / `reset()` / `disable()` - the bus clock and
  the RCC reset pulse; `disable()` IS `reset()` (2.8.1);
- `bus_clock(on)`, `kernel_clock(LptimClock)` and its readback,
  `kernel_clock_running()` (the read 26.4.5's filter rule needs),
  `irq()`, `exti_line`, `dmamux_generator_input`, `regs()`;
- `configure(const LptimConfig&)` - CFGR + CFGR2 + IER in one verb,
  refused while enabled and for a configuration `lptim_config_valid()`
  rejects; `configure<cfg>()` is the compile-time twin whose
  `static_assert` names the rule broken;
- `enable()`, `enabled()`, `start_continuous()`, `start_single()`;
- `set_cmp` / `set_arr` (the STORE) against `cmp_ok` / `arr_ok` and
  `wait_cmp_ok` / `wait_arr_ok` (the OBSERVATION) - separate verbs
  because 26.4.11 makes the handshake the caller's to spend;
- `count()` - two consecutive reads that agree (26.7.8), bounded,
  `std::optional`; `count_raw()` - one read, for the RSTARE case where
  26.4.14 makes the double read impossible;
- `reset_count()` (COUNTRST, refused while one still stands - 26.7.5's
  Caution has no hardware behind it) and `reset_on_read(bool)` (RSTARE);
  the two mechanisms are exclusive by 26.4.14's Warning and nothing in
  the silicon prevents using both;
- `status()`, `clear_flags(mask)`, `clear_flags_raw(mask)`, `isr()`;
- `wake_line(bool)`, `pending_wake()`, `debug_freeze(bool)`.

`lptim_config_valid(n, cfg)` is the constexpr checker both twins use. It
refuses CKPOL = 11; both edges with an EXTERNAL clock (26.4.12 gives an
externally clocked counter one edge or the other); COUNTMODE = 1 with a
prescaler other than /1 (26.4.12); encoder mode on an instance without
it, with an external clock or with a prescaler other than /1 (table 135,
26.4.15's Caution); a trigger row or input code that does not exist on
this instance and this part; and the UP/DOWN interrupt enables where
encoder mode is absent (26.7.3's notes).

ONE REFUSAL HAS NO COMPILE-TIME TWIN AND CANNOT HAVE ONE: 26.4.5 wants
an internal clock present before a glitch filter is switched on, and
whether the selected kernel clock is RUNNING is an RCC fact. So a
nonzero CKFLT or TRGFLT is refused by `configure()`, where
`kernel_clock_running()` can be asked, and not by the constexpr checker.

`clear_flags()` IS ES0548 2.8.2 AS CODE: it refuses (false, nothing
written) when IER is nonzero and the core is in thread mode
(`__get_IPSR() == 0`), and `isr()` does the erratum's ORDER itself - the
pending flags whose interrupt is DISABLED first, then the enabled ones,
returning only the second group so a shared vector can answer "not
mine". With IER == 0 the erratum's own precondition is absent and the
register is an ordinary write-one-to-clear again, which is what makes
the polling tasks legal rather than lucky.

`LptimPad<sel>` hands a pad to IN1, IN2, ETR or OUT. The AF number is
the DATASHEET'S and no symbol of the device header can check it - the
bench is the only check there is.

The tasks are thin: `LptimPwm<L, top>` (a `PwmChannel`),
`LptimPeriodicTick<L>`, `LptimCounter<L>` (free-running, 32-bit through
an ARRM-accumulated high word), `LptimPulseCounter<L>` (both
arrangements of 26.4.12), `LptimTimeout<L>` (26.4.9) and
`LptimEncoder<L>` (LPTIM1 only, `static_assert`ed).

## How to use it

A periodic interrupt that survives a Stop, on the crystal:

```cpp
#include "stm32g0/lptim.hpp"

using Beat = brio::LptimPeriodicTick<brio::Lptim<1>>;

brio::Lptim<1>::init();
brio::Lptim<1>::kernel_clock(brio::LptimClock::lse);
Beat::setup(brio::LptimPrescaler::div32, 1023);   // 1 Hz off 32768/32
brio::Lptim<1>::wake_line(true);                  // EXTI 29, or no wake
brio::Nvic::enable(brio::Lptim<1>::irq());

extern "C" void TIM6_DAC_LPTIM1_IRQHandler() { (void)brio::Lptim<1>::isr(); }
```

A PWM output on LPTIM1_OUT, as `util/pwm_channel.hpp`'s fourth
implementation:

```cpp
constexpr brio::PinSel out{'B', 0, brio::PinFunction::af5};   // LPTIM1_OUT
using Lamp = brio::LptimPwm<brio::Lptim<1>, 999>;

brio::Lptim<1>::init();
brio::Lptim<1>::kernel_clock(brio::LptimClock::pclk);
brio::LptimPad<out>::claim();
Lamp::setup(brio::LptimPrescaler::div128);
Lamp::duty(250);            // a quarter, landing at the next period end
```

A pulse counter with every oscillator off - the input IS the clock:

```cpp
using Pulses = brio::LptimPulseCounter<brio::Lptim<1>>;

brio::Lptim<1>::init();
brio::LptimPad<in1>::claim_input(brio::PinPull::up);
Pulses::setup_external(brio::LptimClockPolarity::falling);
// 26.4.12: the first five active edges after the enable are LOST, and
// the bench confirms it is exactly five.
const auto n = Pulses::count32();
```

A timeout that a hardware trigger keeps feeding (26.4.9):

```cpp
using Watch = brio::LptimTimeout<brio::Lptim<1>>;

Watch::setup(brio::LptimTrigger::rtc_alarm_a,
             brio::LptimTriggerEdge::rising, 10'000);
// A software trigger cannot feed a timeout: TRIGEN != 00 is required,
// so setup() refuses LptimTriggerEdge::software.
```

## Bench findings

Every number below is `test_stm32_lptim`'s, measured against the RTC's
sub-second counter on the LSE crystal (PREDIV_A 0: a 30.5 us stopwatch)
or against SysTick's cycle count at 64 MHz. The suite is WIRELESS: all
four LPTIM1 signals are on port B at AF5 and each pad is walked by its
own internal pull, which letter a proves before anything rests on it.

**A pad handed to an LPTIM INPUT function still follows its own pull.**
PB5, PB6 and PB7 walk between the rails under AF5 exactly as they do as
plain inputs. (The SAM found that a DRIVING peripheral function takes
the output driver and the pull with it; an INPUT function does not, and
that is what makes this whole chapter measurable with no wire.)

**All four kernel clocks drive the counter**, each inside its own band:
LSE 32740 counts a second against the crystal's 32768 (the RTC judging
the LPTIM on the same crystal - a consistency check, not a frequency),
LSI 32660 against the 32586 the platform, tim and rtc suites saw, HSI16
125210 x 128 and PCLK 501040 x 128 against 64 MHz.

**26.4.13's and 26.4.7's latencies are REAL KERNEL CLOCKS, not APB
ones.** On LSE, `enable()` plus the first ARR write to ARROK costs
125..152 us and CNTSTRT to the counter actually moving 122 us; the same
two steps on PCLK cost 2 us and 1 us.

**26.4.11's write handshake, in CPU cycles - a number the chapter never
gives**: on an LSE kernel clock a CMP write reaches CMPOK in about 4600
to 6000 cycles at 64 MHz (72..93 us, two to three LSE periods) and an
ARR write in about 5800; on PCLK both cost 131..136 cycles. This is what
sizes the sleep site's minimum alarm distance.

**A FORBIDDEN WRITE IS NOT ONE THING ON THIS FAMILY** (the analog
campaign's finding, met again). 26.7.4 says CFGR "must only be modified
when the LPTIM is disabled" - and a CFGR store made while the block is
ENABLED LANDS: the register reads back what was written. What the
chapter forbids is what the COUNTER then does with it, not the store.
So the refusal has to be the driver's, and `configure()` returns false
and writes nothing rather than pretending the silicon refused.
A CNTSTRT written while DISABLED, by contrast, really is discarded by
hardware (26.4.7): CR reads 0 afterwards and a later enable starts
nothing.

**CFGR2 KEEPS EIGHT BITS.** Written all ones it reads back 0xFF, so
IN1SEL and IN2SEL are FOUR bits wide as the device header declares them
(`CFGR2[3:0]` and `CFGR2[7:4]`) and not the two bits 26.7.9 draws - the
two upper codes of each are simply unnamed. A documentary dispute
settled by experiment, in the header's favour.

**26.7.8's double read has a SECOND reason the chapter does not name.**
On an asynchronous kernel clock two consecutive CNT reads really do
disagree - 3 to 9 readings in 2000 at LSE - which is the reason 26.7.8
gives. But with the counter clocked at ONE COUNT PER CPU CYCLE (PCLK,
prescaler /1) the two reads NEVER agree, 2000 times in 2000: not because
the value is incoherent but because it has MOVED. `count()` gives
nothing there and `count_raw()` is the only reader that means anything
on a fast synchronous clock.

**The eight prescaler ratios are exact** - the same window gives 4096
counts (+/- 22 per mille at /1, where the window is only 4096 CPU
cycles, and 0 per mille from /32 up).

**Both of 26.4.8's on-the-fly switch sentences hold**: a CNTSTRT written
into a stopped one-shot restarts it continuously, and a SNGSTRT written
into a running continuous count stops it at the next ARR match.

**COUNTRST costs 2 extra counts** before it lands, against the three
kernel clocks 26.4.14 predicts - the chapter's "a few extra pulses" with
a number on it. RSTARE works as described, and with it set the double
read is impossible by construction.

**PRELOAD measured as what it is**: with it clear, an ARR halved
mid-flight makes the VERY NEXT period the new one (49 ms then 49); with
it set, the next period is still the old one (99 ms) and only the one
after is new (49) - 26.4.11 exactly.

**THE WAVEFORM ARITHMETIC, WHICH 26.4.10 DESCRIBES IN WORDS AND NEVER
PRINTS.** Measured off the pad:

- a PERIOD is **ARR + 1** counter ticks (ARR = 499 on a 500 kHz counter
  gives 1000 Hz, where ARR ticks would give 1002);
- the HIGH TIME is **ARR - CMP + 1** ticks - ONE MORE than the
  chapter's two sentences read on their own - so CMP = 0 is a FULL duty
  and not one tick short of it: 1000, 751, 501, 251 and 101 per mille
  for CMP 0, 249, 499, 749 and 899 at ARR 999;
- and at the pair 26.4.10 forbids (CMP >= ARR) the output is a **FLAT
  LOW**, not the one tick of duty the formula would extrapolate to, not
  a fault, and not undefined-looking.

`LptimPwm` maps duty v out of `max` = ARR onto CMP = max - v, which
makes both endpoints exact by two different routes: v = max puts CMP at
0 and the output high for the whole period, and v = 0 lands on the
forbidden pair, whose flat low IS zero duty. Measured ladder: 0, 251,
500..502, 751..752, 1000 per mille off the pad for 0, 250, 500, 750,
999. WAVPOL inverts the waveform and nothing else. One-pulse leaves the
output low for ever after its pulse; set-once leaves it at its last
level, which is the whole difference between them.

**26.4.10's "up to the LPTIM clock frequency divided by 2" is exact**,
and it is measured with no pad, no peripheral and no CPU in the loop:
ARR = 1 on the 32768 Hz crystal drives the DMAMUX request GENERATOR
through table 56's trigger input 20, and a DMA channel with no
peripheral counted 3277 edges in 200 ms = 16385 Hz against the kernel
clock's half, 16384. The same claim at the PCLK extreme (32 MHz of
output) is DECLINED: no counter this board can spare resolves it and the
DMA cannot serve requests that fast.

**26.4.12's lost edges are EXACTLY FIVE.** With CKSEL = 1 on a
pull-walked pad, 20 applied edges are counted as 15 - on the rising edge
and on the falling edge alike. With COUNTMODE = 1 (the internal clock
SAMPLING the same input) all 20 are counted and nothing is lost at the
start, which is the practical difference between the two arrangements.

**CKFLT is a real threshold and not a formality**: at one sample per
30.5 us on an LSE kernel clock, `samples8` rejects ten 60 us blips
(0 counted) and passes ten 2 ms pulses (10 counted), while with the
filter off the blips are counted like anything else.

**The internal routes work with no pad on the timer's side at all**:
IN1SEL = COMP1_OUT counts 12 comparator flips as 12 (the comparator's
own input being a precharged PA1), and the RTC's ALARM A and COMP1_OUT
each start the counter through the trigger multiplexer.

**A trigger arriving while the counter already runs is ignored AND ITS
FLAG IS NOT SET** - 26.7.1's easily-missed sentence, measured: EXTTRIG
stays clear while the counter keeps running through the second edge.
With TIMOUT set instead, the trigger RESTARTS the counter, so a compare
match is the statement that no trigger arrived in time (fed every 3 ms a
10 ms timeout never expires; starved for 30 ms it does).

**All three encoder sub-modes count exactly as table 144 says**: 32
quadrature transitions walked by two internal pulls move the counter by
16 in sub-modes 1 and 2 and by 32 in sub-mode 3, in both directions, and
the UP/DOWN flags mark each CHANGE of direction - which is not the same
thing as a direction.

**Through a Stop, 26.5's table 145 is exact - AND HSI16 IS NOT A STOP
CLOCK FOR A COUNTER.** Across a real Stop 1 the counter keeps every
count on LSE (7431 counts of a 226 ms Stop) and on LSI (7734 of 240 ms),
and stops dead on PCLK (10 counts of 239 ms). On HSI16 IT ALSO STOPS
(5 counts of 239 ms). 5.3 lists the LPTIMs among the peripherals that
can REQUEST HSI16 in Stop, but a counter that merely COUNTS makes no
such request - only LSE and LSI, the two clocks table 145 names, keep it
running. (And ES0548 2.2.4 breaks clock requests on a divided HSI
anyway, so an LPTIM on HSI16 as a wake source is a configuration this
driver names and does not recommend.)

**A COMPARE MATCH IS A PER-LAP EVENT AND NOT A ONE-SHOT.** The handler
runs once for every time the counter passes CMP - 10 times in 9 laps on
LSE, 50 in 50 on PCLK - which is why a compare left standing keeps
waking a device for ever, and why the third sleep site has to say so.
The flag clear LANDS AT ONCE even on a 32 kHz kernel clock: LPTIM_ISR
already reads the flag gone in the instruction after the ICR store, so a
handler is entered once per event and not once per event plus a spurious
re-entry.

**The wake from both deep rungs works through EXTI line 29** and lands
where the counter said it would: a compare placed 512 counts ahead on
LSE/32 woke the core after 503.6 ms of wall from Stop 0 and 503.5 ms
from Stop 1, against 500.0 ms predicted, with exactly one interrupt each
time. THE ORDER MATTERS AND COST A MEASUREMENT ITS MEANING: CMP comes
out of reset at ZERO, so a counter started before the compare is placed
matches it on its very first tick and spends the interrupt before the
sleep. The compare is placed first, here and in the sleep site.

## Not covered yet

Driver gaps - things chapter 26 has and this file does not:

- nothing. Every register, field and mode of the chapter is implemented,
  including the two upper (unnamed, unconnected) codes of each input
  multiplexer, which the driver refuses rather than offers.

Implemented but not bench-verified:

- **LPTIM2 on silicon.** Everything measured here was measured on
  LPTIM1, because it is the instance with the encoder and the second
  input and because its four signals land on four free pads. LPTIM2's
  presence, vector, EXTI line and refusals are checked by the family
  fixture and the negatives; its counter has never been run.
- **TRGFLT**, the trigger filter, measured only through CKFLT's twin -
  the two fields share the vocabulary and 26.4.5 describes them
  together, but a filtered TRIGGER was not staged separately.
- **The trigger rows this board cannot reach**: TAMP1, TAMP2 and
  TAMP_TRG3 (arming a tamper input erases the backup registers this
  stratum leans on - the same decline rtc.md makes), and COMP2_OUT and
  COMP3_OUT as triggers (COMP1_OUT is measured; the other two are the
  same multiplexer row on a comparator this suite does not flip).
- **Set-once mode's discarded triggers** (26.4.8's figure 274): the
  waveform is measured, the "any subsequent trigger event is discarded"
  half is not.
- **`debug_freeze()`**: the DBG block's own gate is opened and the bit
  is written, but nothing here halts a core to watch the counter freeze.

Declined, with the reason:

- **The output rate at the PCLK extreme** (32 MHz with ARR = 1): no
  counter this board can spare resolves it and the DMAMUX generator
  cannot serve requests that fast. The claim is measured at the LSE end
  instead, where it is exact.
- **ES0548 2.8.1 is NOT STAGED and no verdict pretends it was.**
  Reproducing it needs the very `CR.ENABLE` clear this driver has no
  verb for; its own description calls the occurrence "very low"; and its
  failure mode - a wake-up signal frozen active - would leave the board
  unable to enter Stop at all. It is answered structurally and recorded
  as unreachable, not as disproved.
