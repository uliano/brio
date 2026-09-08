# Low-power timers (STM32G0)

> **PROVISIONAL.** Chapter 26 is implemented whole - both instances,
> every register and every field, the four kernel clocks, both counter
> modes, the trigger and input multiplexers, the three waveforms, the
> glitch filters, encoder mode, the two counter resets and the wake
> line - and the tasks over it give `util/pwm_channel.hpp` its fourth
> silicon, `util/power.hpp` its third sleep site on this target, and
> the kernel its first TICKLESS timebase (`LptimTicker`,
> [platform.md](platform.md)). What is still missing is in "Not
> covered yet".

Documents of record: RM0444 Rev 6 - LPTIM ch. 26, the kernel-clock
multiplexer 5.4.21 (RCC_CCIPR), the bus enable and reset 5.4.19/5.4.11,
the low-power modes 5.3 and 26.5, the EXTI line map table 65, the
interrupt table 12.3 (table 61), the DMAMUX trigger inputs table 56;
DS13560 Rev 5 tables 13..24 (the AF numbers of the LPTIM pads); errata
ES0548 Rev 3 items 2.8.1 and 2.8.2, read on the bench chip's revision Z
column. Driver: `stm32g0/lptim.hpp`; the third sleep site over it is
`Stm32g0LptimTimedSleepSite` in `stm32g0/sleep.hpp` and is documented in
[pwr.md](pwr.md); the tickless kernel timebase over it is
`LptimTicker` in `stm32g0/lptim_ticker.hpp` and is documented in
[platform.md](platform.md). The per-instance presence, vector, EXTI and
DMAMUX facts come from `stm32g0/device_tables.hpp`. Bench suites:
`test_stm32_lptim` (10 letters, 82 verdicts, wireless) and
`test_stm32_tickless` (the timebase). Family fixtures
`test/family_stm32g0/lptim.cpp` and `lptim_ticker.cpp` plus their
negatives under `tools/check_stm32g0.sh` (the third sleep site's own
two live with `test/family_stm32g0/sleep.cpp` - see [pwr.md](pwr.md)).

## What the silicon does

A 16-bit up-counter with a 3-bit power-of-two prescaler, one compare and
one auto-reload, whose point is that IT DOES NOT NEED THE REST OF THE
CHIP. Two things follow from that and shape everything else:

**The counter's clock is chosen twice.** `RCC_CCIPR.LPTIMnSEL` picks the
KERNEL clock - PCLK, LSI, HSI16 or LSE, the same four for both instances
on every G0x1 of this pack (the x0 value line has NO LPTIM at all: no
`LPTIM1_BASE`, and `Lptim<n>` and its sleep site do not exist there,
while this chapter's vocabulary still compiles) - and then `CFGR.CKSEL` decides whether the
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
`LptimEncoder<L>` (LPTIM1 only, `static_assert`ed). The seventh lives
in its own header, `stm32g0/lptim_ticker.hpp`: `LptimTicker<cfg>`, the
kernel timebase that does not stop - the counter undivided on the
crystal, the tick its count shifted, the loop's next deadline placed in
CMP by the platform's `idle_until()`; [platform.md](platform.md) owns
it.

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
four LPTIM1 signals are on port B at AF5 and LPTIM2's three are PC0, PC3
and PD6 at AF2, and each pad is walked by its own internal pull - which
letter a proves for the first four and letter j for the last three
before anything rests on them.

**A pad handed to an LPTIM INPUT function still follows its own pull.**
PB5, PB6 and PB7 walk between the rails under AF5 exactly as they do as
plain inputs, and so do PC0 and PC3 under AF2. (The SAM found that a
DRIVING peripheral function takes the output driver and the pull with
it; an INPUT function does not, and that is what makes this whole
chapter measurable with no wire.)

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
sizes the sleep site's minimum alarm distance. **AND THE LATENCY SCALES
WITH THE PRESCALER** (test_stm32_tickless letter x): the same CMP write
lands in 2.0..2.8 ms at prescaler /32 - two to three PRESCALED counts,
not two to three kernel clocks - so "a few LSE periods" is a fact of
the undivided counter only, and a compare placed three counts out at
/32 is missed for a whole lap. The same letter measured WHERE THE MATCH
FIRES: CMPM rises at the count edge AFTER equality, CMP + 1, at every
distance tried (4, 5, 6, 10, 40 counts), and a compare that lands on
top of its own equality fires at CMP + 2. Both facts are what the
tickless timebase is built on (the counter undivided, the compare at
the deadline's first count less one, a six-count floor); the sleep
site's own "+1 count for the phase" rule already absorbed the first
of them without naming it. **A compare equal to ARR matches** like any
other value (letter b: CMP = 0xFFFF for a lap, one CMPM).

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

**And so is TRGFLT, which is the same field on the other input** (letter
j, on LPTIM2's own ETR pad and an LSE kernel clock): with the filter off
the first 60 us blip on PC3 is a trigger like any other and the counter
starts; with `samples8` ten of them leave the counter at ZERO with
EXTTRIG never set, and a 2 ms pulse starts it. 26.4.5 describes CKFLT and
TRGFLT together and this is the measurement that says they behave alike.

**The internal routes work with no pad on the timer's side at all**:
IN1SEL = COMP1_OUT counts 12 comparator flips as 12 (the comparator's
own input being a precharged PA1), and the RTC's ALARM A and COMP1_OUT
each start the counter through the trigger multiplexer. On LPTIM2,
IN1SEL = 3 - the OR of the two comparators, a code table 140 leaves
unconnected on LPTIM1 - carries the same 12 flips of COMP1 with COMP2
quiet.

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

### LPTIM2 (letter `j`)

The second instance is the same `LPTIM_TypeDef` at another address and
the manual's tables say it is not the same peripheral. Every asymmetry
above is now measured on silicon rather than only stated:

- **its own three pads, and they are on another port and another
  function**: PC0 = LPTIM2_IN1, PC3 = LPTIM2_ETR and PD6 = LPTIM2_OUT,
  all AF2 (DS13560 tables 17 and 18). All three follow their own pull
  as plain inputs, and the two inputs still do under AF2;
- **its own CCIPR field**: all four codes of `LPTIM2SEL` drive the
  counter - LSE 32740, LSI 32680, HSI16/128 125260 and PCLK/128 501240
  counts a second, each inside its own band, which is what says the
  reserve's field position for the second instance is right;
- **the same waveform arithmetic on PD6**: 1000, 752, 498, 248 and 97
  per mille for CMP 0, 249, 499, 749 and 899 at ARR 999, and WAVPOL
  inverting it (253 against 747);
- **table 56's trigger input 21 IS LPTIM2_OUT**: ARR = 1 on the crystal
  gives a request generator 16385 edges a second against the kernel
  clock's half, 16384 - the LPTIM1 measurement repeated on the other
  number the reserve carries;
- **both of 26.4.12's arrangements on PC0**: 20 of 20 edges sampled by
  the internal clock, 15 of 20 with the input AS the clock - the same
  five lost at the start;
- **the two asymmetries as refusals both ways**: `COMP3_OUT` is legal
  on LPTIM1 and refused on LPTIM2 while `TAMP_TRG3` is legal on LPTIM2
  and refused on LPTIM1 (the same TRIGSEL code, two different signals -
  tables 138 and 139), and `COMP2_OUT` and the OR of the two
  comparators are accepted as IN1SEL on LPTIM2 and refused on LPTIM1,
  at compile time and at run time alike;
- **and one vector with two owners.** Table 61 puts LPTIM2 and TIM7 on
  the same line, and the letter makes both speak on it: in one 20 ms
  window `TIM7_LPTIM2_IRQHandler` ran 19 times for LPTIM2's ARRM and 20
  for TIM7's update, each body clearing and returning exactly the flags
  its own enable had asked for.

## On the second silicon

`test_stm32_lptim` runs on the Nucleo-G071RB (DEV_ID 0x460, REV_ID
0x2000) and scores **82/82**, every letter and every verdict the G0B1RE
gives. Both instances, both clocks, the waveform, the encoder, the
filters, the DMAMUX trigger and the two errata legs behave identically.
ONE COMPILE-TIME STATEMENT MOVES WITH THE PART: `lptim_ext_trig5` is
COMP3_OUT on LPTIM1, and a part with no third comparator has no such
signal, so `lptim_config_valid(1, {.trigger = comp3_out})` is FALSE there
- the suite's asymmetry pair is written as the reserve's own fact
(`== comp_present(3)`) rather than as a constant, and holds both ways.
The corresponding ES0418 items are 2.9.1 and 2.9.2, word for word the
G0B1's 2.8.1 and 2.8.2, and both are answered the same way.

## On the third silicon

`test_stm32_lptim` scores **78 of 82** on the Nucleo-G031K8 (DEV_ID
0x466, REV_ID 0x1003). The four not claimed are each a skip by name,
and all four are the PART's: two comparator routes - IN1SEL = COMP1_OUT
and the comparator OR - and the comparator row of the trigger table,
because **this part has no COMP at all**, so the type cannot even be
named and those legs are compiled out; and the "one vector, two owners"
leg, which needs the TIM7 this part has not got (below). The crystal
runs on this board and every LSE row is measured, folded into the same
verdicts it shares on E: the kernel-clock census reads the **LSE at
32740 counts a second**, the LSI at 31430 (against a 32586 nominal, 35
per mille - the slowest die of the desk), HSI16/128 and PCLK/128 at
125240 and 501010; the Stop letter reads 7434 LSE counts across a 226
ms Stop 1; and the third sleep site on the crystal matures a 500 ms
event at 501 ms of wall with six 150 ms rounds at 152 ms, none early.

**BOTH LPTIMs HAVE A VECTOR OF THEIR OWN HERE.** LPTIM1's is shared with
TIM6 and the DAC where those exist and is LPTIM1's alone where they do
not; LPTIM2's is shared with TIM7 or is LPTIM2's own - so letter `j`'s
"one vector, two owners" leg skips (`tim_present(7)` is false) and the
vector verdict is the reserve's derivation itself: `lptim_irq(2)` is not
`lptim_irq(1)`, and it equals `tim_irq(7)` exactly where there is a TIM7.
The suite reaches both handlers through `BRIO_STM32G0_LPTIM1_HANDLER`
and `BRIO_STM32G0_LPTIM2_HANDLER`, which is not decoration: bound by the
G0B1's own names this image would have been DEAD on this part, both
vectors unbound.

**LPTIM2'S THREE PADS ARE A PACKAGE QUESTION**: PD6/PC0/PC3 at AF2 on
the LQFP64s, and **PA4 (OUT), PB1 (IN1) and PA5 (ETR) at AF5** on the
LQFP32, which bonds neither port D nor PC0..PC5 (DS12992 table 12,
tables 13-17 for the functions).

**THE RULER IS THE WALL ON EVERY BOARD OF THIS DESK**, the crystal being
the instrument every number of this document is on. The suite keeps the
other arrangement as a stated rule for a board without one: a root
weighed on TIM16 at boot, every band that quotes the crystal computed
from that number, and the awake windows ruled by the CPU's own cycle
counter so the PCLK-derived ratios stay exact tests under an RC wall.

## Not covered yet

Driver gaps - things chapter 26 has and this file does not:

- nothing. Every register, field and mode of the chapter is implemented,
  including the two upper (unnamed, unconnected) codes of each input
  multiplexer, which the driver refuses rather than offers.

Implemented but not bench-verified:

- **LPTIM2's ENCODER-shaped half, which does not exist**: nothing is
  left to run there - the instance has no encoder and no second input,
  and letter j measures the refusals. What letter j does NOT repeat on
  the second instance is what would only say the same thing twice: the
  prescaler ladder, PRELOAD, the two counter resets, the timeout
  function and the Stop behaviour are LPTIM1's letters and are facts of
  the shared design, not of an instance.
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
