# Comparators (STM32G0)

> **PROVISIONAL.** All of chapter 18's register surface is implemented -
> the three input multiplexers, the hysteresis and the two speeds, the
> polarity, the blanking sources, the window mode and its output
> selector, the lock and the EXTI line each comparator publishes - and
> everything a bench with no wires can reach is verified. A comparator's
> non-inverting input is a pad and only a pad, and this family
> disconnects a pad's own pull the moment it goes analog; what a FREE
> pad can nevertheless be made to do is measured below, and what it
> still cannot - a threshold measured against a level something is
> HOLDING - is listed in "Not covered yet" with the numbers that decline
> it.

Documents of record: RM0444 Rev 6 ch. 18, with the EXTI line table
13.5.1, the vector table 12.3 (table 61) and TIM1_TISEL / TIM1_AF1
(21.4.27, 21.4.28); DS13560 Rev 5 table 12 (the input pads) and table 68
(offset, hysteresis, propagation delay); errata ES0548 Rev 3, which has
**no item touching the comparators**. Driver: `stm32g0/comp.hpp`; the
per-part presence and the EXTI line numbers come from
`stm32g0/device_tables.hpp`. Bench suite: `test_stm32_analog` letters i
(15 verdicts), m (7 - the analog questions) and n (8 - the other two
comparators, the output on a pad, the blanking sources), shared with the
ADC, the DAC and the reference buffer.
Family fixture `test/family_stm32g0/comp.cpp` plus three negatives under
`tools/check_stm32g0.sh`.

## What the silicon does

**Three comparators, two, or none, and the header says which.** 18.1:
COMP3 is the G0B1xx/G0C1xx's alone, and the G031 class has no comparator
at all - each device header saying exactly that by declaring or not
declaring `COMPn_BASE`. So `Comp<3>` is a compile error on a G071 and
`Comp<1>` is one on a G031, with no hand-kept list behind either.

**One CSR each, and it lives inside SYSCFG.** 18.3.3 says two things
that read as contradictory - "there is no clock enable control bit
provided in the RCC controller" and "reset and clock enable bits are
common for COMP and SYSCFG" - and the address map settles it:
`COMP1_BASE` is `SYSCFG_BASE + 0x200`, so RCC_APBENR2.SYSCFGEN is what
makes the register readable and writable, while the comparator's own
polarity and output logic keep working without it (which is what makes a
comparator useful in Stop mode). `Comp<n>::init()` opens that gate and
`release()` does not close it - SYSCFG is shared.

**THE ONLY ANALOG SOURCE INSIDE THE CHIP REACHES THE MINUS INPUT.**
Tables 94, 96 and 98 give the inverting input VREFINT, its quarter, half
and three-quarter taps, BOTH DAC channels and three pads. Tables 93, 95
and 97 give the non-inverting input three PADS and "open" - and no
internal signal at all. So a THRESHOLD can be produced with no wire and
a SIGNAL cannot.

**And this family will not lend a pull to an analog pad.** 7.3.13: in
analog configuration "the weak pull-up and pull-down resistors are
disabled by hardware". The SAM C21's analog letters walked a pad between
the rails with its own pull; that technique has no twin here, and it is
measured rather than assumed (see the findings).

**The input tables are per instance and are not a pattern.** COMP1's
plus inputs are PC5/PB2/PA1, COMP2's PB4/PB6/PA3, COMP3's PB0/PC1/PE7 -
and PE7/PE8 exist only where port E is bonded, which is the same class
boundary reached from the GPIO side. `comp_positive_pin()` and
`comp_negative_pin()` publish the tables as data and `config_valid()`
refuses a selection whose pad this device does not bond.

**WINMODE's partner is NOT n + 1.** 18.3.5 names two pairs
(COMP1+COMP2 and COMP2+COMP3) and 18.6.1 says which way each borrows,
one register description at a time: COMP1 takes COMP2_INP, **COMP2 takes
COMP1_INP**, COMP3 takes COMP2_INP. A driver that assumed the next
instance up would refuse a legal COMP2 window and allow an impossible
one, so `comp_window_partner()` exists and every instance publishes its
own.

**LOCK is one-way until a reset.** 18.3.4: setting it makes the whole
register read-only INCLUDING the lock bit, and only an MCU reset clears
it. The verb is offered because the safety case is what the bit is for,
it is spelled `lock()` so nobody reaches it by accident, and every
configuring verb refuses while it stands rather than storing into a
register the silicon ignores. **The suite never sets it.**

**There is no interrupt of its own.** 18.5 sends each comparator's
output to an EXTI line - 17, 18 and 20, all three CONFIGURABLE, so a
sense must be chosen before anything is pending - and the vector is the
ADC's, shared by all three (table 61). This driver PUBLISHES its line
number and includes no EXTI header: the fabric driver owns the fabric
and the peripheral owns its own vocabulary.

## The types and the verbs

`stm32g0/comp.hpp` is the reference. `CompConfig` +
`comp_config_valid(n, cfg)` carry the whole configuration and the
chapter's refusals (a Reserved PWRMODE code, a blanking bit past the
five, a DAC threshold on a part with no DAC, a pad this package does not
bond, a window with no partner); `Comp<n>::configure()` writes the CSR
and leaves the comparator disabled, `claim_inputs()` puts the pads the
configuration names into analog mode, and `enable(true)` is 18.5's own
LAST step, after the EXTI line is armed. `value()` is the status bit,
`exti_line` and `irq()` are what an application binds, and `lock()` /
`locked()` are apart from everything else.

```cpp
constexpr brio::CompConfig cfg{
    .positive = brio::CompPositive::input2,          // COMP1_INP2 = PA1
    .negative = brio::CompNegative::dac_channel1,    // no pad at all
    .hysteresis = brio::CompHysteresis::medium,
};
brio::Comp<1>::init();                 // the SYSCFG gate
brio::Comp<1>::claim_inputs(cfg);
brio::Comp<1>::configure(cfg);
brio::Exti::sense(brio::Comp<1>::exti_line, brio::ExtiSense::both);
brio::Exti::interrupt(brio::Comp<1>::exti_line, true);
brio::Nvic::enable(brio::Comp<1>::irq());
brio::Comp<1>::enable(true);
```

## Bench findings

Measured by `test_stm32_analog` letter i on an STM32G0B1RE at 3.3 V,
silicon revision Z.

**THE STIMULUS IS A PRECHARGED PAD, and its lifetime is measured.**
With no internal source able to reach a plus input and no pull available
in analog mode, the pad is driven to a rail by GPIO and then handed to
the comparator; the node's own capacitance holds it there. Measured:
precharged high the comparator reads 1 and precharged low it reads 0,
and **the node stays above half VREFINT for longer than the 100 ms cap
the letter puts on the measurement** - so this is a technique with four
orders of magnitude of margin and not a race.

**7.3.13 is literal, and here is the number.** A pad in analog mode with
its pull-up ASKED FOR converts to 960 counts of 4096 and with the
pull-down to 1056 - a floating node drifting, not two rails. The SAM's
pull-walked analog stimulus does not exist on this family.

**All four VREFINT taps work and POLARITY inverts what VALUE reports.**
Each of 1/4, 1/2, 3/4 and full VREFINT sits between the rails (high on a
precharged 3.3 V, low on a precharged ground) and reads back from
INMSEL; with POLARITY set the answer flips, because 18.6.1 puts VALUE
after the polarity selector.

**THE BLANKING WINDOW IS A REAL GATE, measured with no pad and no
wire.** TIM1's OC4 forced ACTIVE while the comparator's input says high
drives VALUE to 0, and forcing it inactive again gives the answer back:
1 -> 0 -> 1. Two things had to be right and the first version of the
letter had neither: the blanking source must rise AT a live comparator
(a level that was already there when the comparator was enabled produced
nothing), and TIM1's channel needs CCER's enable and MOE for its OCREF
to reach the comparator.

**A WINDOW COMPARATOR WITH ONE PAD.** COMP2's WINMODE takes COMP1's plus
input (18.3.5), so one precharged node is compared against two
thresholds at once - COMP1 at 3/4 VREFINT and COMP2 at 1/4 - and both
comparators say "above" together and "below" together. The **INSIDE**
state is DECLINED in print: it needs the node held between the two
thresholds, and no source inside this chip can do that.

**WINOUT DID NOTHING THIS BENCH CAN SEE, and that is recorded rather
than explained.** 18.6.1 says COMP1_CSR.WINOUT selects "COMP1_VALUE XOR
COMP2_VALUE". Staged in figure 69's own arrangement - COMP1 owning the
pad with WINMODE clear and WINOUT set, COMP2 borrowing it with WINMODE
set and WINOUT clear, both enabled on the same node - **COMP1's VALUE
still follows the node exactly**, and it goes on doing so when the
partner's polarity is inverted so the pair disagrees at BOTH rails
(COMP1 reads 1/0 while COMP2 reads 0/1). The bit is demonstrably
written and standing (CSR 0x40004221). Nor does it reach the OUTPUT: the
same four rail changes fire COMP1's EXTI line four times with WINOUT set
and three with it clear, where a constant xor would have gone silent.
What would settle it is a node held BETWEEN the two thresholds, where
the pair disagrees on the SIGNAL and not on a polarity bit - one wire
from PA4 (DAC1_OUT1) to PA1, the same wire the INSIDE state wants.

**The output leaves the comparator two ways at once, and both are
counted.** Six rail changes on the node produce **5 interrupts on EXTI
line 17** (2 rising and 3 falling, in the two SEPARATE pending
registers) and **5 captures on TIM1's TI1**, which TIM1_TISEL points at
COMP1's output (21.4.28) - two mechanisms sharing nothing but the
comparator, agreeing on what happened.

**Hysteresis and the speed selector** are written and read back; DS13560
table 68 puts the three hysteresis levels at 10, 20 and 30 mV typical
and the two speeds at 30 ns and 0.3 us of propagation delay. **Their
analog effect is not measured** - both need an input that MOVES through
a threshold, which is the same missing wire.

**COMP3 answers, and it now RUNS** (letter n). Its two bonded plus pads
PB0 and PC1 read high and low against half of VREFINT, and its own EXTI
line 20 delivered eight interrupts for eight edges through the vector it
shares with the ADC and the other two comparators - so COMP3 is a signal
path here and not only a register. Its third plus input is PE7, which
the driver reports VALID because this DEVICE has a port E; whether this
PACKAGE bonds that pad is a per-package table this stratum does not have
([port.md](port.md) carries the gap), so the pad is named and left
alone. COMP2 likewise runs on its OWN pads - PB4 as INPSEL 0 and PB6 as
INPSEL 1 - where every other letter reaches it by borrowing COMP1's
through WINMODE.

**The lock is clear and stays clear** on all three, which is what a
suite that must be re-runnable can say about a bit whose only exit is a
reset.

### A free pad is a stable analog source (letter `m`)

The stimulus letter i uses is a pad driven to a RAIL and released, which
is enough for every logical question and for none of the analog ones.
Letter m starts from the same release and then watches:

**A FLOATING PAD DOES NOT STAY AT THE RAIL - IT RELAXES TO AN
EQUILIBRIUM AND SITS THERE.** Released from VDD and read continuously,
PA1 falls through 1440, 1499, 1501, 1519, 1514 and 1511 ADC counts over
eighteen milliseconds and stops: about **1.22 V, a third of the way up
the supply**, reproducible to a count between one release and the next
(1511 then 1512). Nothing in any chapter names it and it is a BOARD
fact, not a silicon one - it is where that pad's own leakage paths
balance, and another pad or another board will settle somewhere else.
The CONTROL says the reading is not passive: ten milliseconds with the
converter looking elsewhere move the node by five hundred counts, so the
conversions are part of what holds it.

**So the DAC IS a threshold on silicon.** With `DacMode::internal_
unbuffered` the converter drives the on-chip peripherals and no pad at
all - PA4 is not even claimed - and COMP1's INMSEL 4 compares the
settled node against it: VALUE 1 with the DAC at code 0 and 0 at 4095.
`CompNegative::dac_channel2` goes the same way as the upper limit of the
window below.

**THE WINDOW'S INSIDE STATE, which letter i can only decline.** With
COMP1 against DAC channel 1 and COMP2 against DAC channel 2 through
WINMODE, one settled node reaches ALL THREE states - above both limits,
between them, below both - by moving the WINDOW instead of the signal.
18.3.5's window comparator is complete on this desk.

**The two propagation delays differ by a real, measured time.** A DAC
step of 300 codes (242 mV, table 68's own 100 mV of overdrive and more)
across the threshold is answered in **687..1062 ns in high-speed mode
and 1109..1218 ns in medium-speed mode** over several runs, a difference
of **406..484 ns** where table 68 prices the gap at 270. The absolutes
are the DAC's settling plus the comparator plus the poll loop and are
NOT the comparator's; the difference is, because everything else does
the same thing twice.

**AND THE OFFSET AND THE HYSTERESIS ARE DECLINED, with the number that
declines them.** The band between a falling crossing and a rising one is
measured at all four HYST codes - and **with HYST CLEAR it is 88..93 mV,
where it should be zero and where table 68's LARGEST hysteresis is 30**.
So the band this instrument reports is the NODE's own motion and not the
comparator's, and no arrangement of the four numbers is a measurement of
hysteresis. The offset goes the same way: with HYST clear the crossings
sit 194..199 mV from the ADC's reading of the same node, which is two
instruments reading different parts of one waveform. The reason is the
settled node itself: it is held up by the conversions that read it, so
it is a sawtooth - the ADC reports its sampling instant and the
comparator watches all of it. **One wire from PA4 to PA1 settles both**,
and this suite does not pretend to have found a way round it.

### The output on a pad, and the blanking sources (letter `n`)

**COMPx_OUT is alternate function 7** (DS13560 table 13) and PA6 carries
COMP1's output: the pad's own input register follows VALUE at both
rails, and the EXTI line of THAT pad counted six edges for three round
trips of the input. That extends the exti campaign's finding - a line
sees a pad its owner is driving - from the CPU to a PERIPHERAL.

**Three more of 18.6.1's five blanking sources are real gates**:
TIM2_OC3, TIM3_OC3 and TIM15_OC2 each drive VALUE low while the input
says high when their channel is forced active, and give the answer back
when it is released - measured the way letter i measures TIM1_OC4, with
no pad and no counting. **The fifth, TIM1_OC5, is not reachable from
here and is not pretended to be**: it is the CCR5 channel, which
`tim.hpp` deliberately does not build ([tim.md](tim.md)'s own gap). The
BLANKSEL bit for it is still written and read back, because a mask this
driver refused to select would be a claim about a timer's channels and
not about this chapter.

## Not covered yet

**Implemented but not bench-verified:**

- **The OFFSET and the three HYSTERESIS levels**, declined above with
  the measurement that declines them: the instrument's own floor is
  88..93 mV where the largest effect it would measure is 30. Both want
  the plus input held at a chosen voltage by something that is not also
  reading it - one wire from PA4 to PA1, and this campaign is wireless
  by construction.
- WINOUT, which did nothing measurable (above) and is recorded as such.
- **TIM1_OC5 as a blanking source**, because `tim.hpp` builds four
  channels per timer and TIM1's fifth and sixth are the combined-PWM
  pair it does not build ([tim.md](tim.md)). The BLANKSEL bit is written
  and read back; the GATE is not seen.
- COMP3's third plus input PE7, which the driver reports valid on this
  DEVICE and which this PACKAGE may not bond - a per-package pin table
  this stratum does not have ([port.md](port.md)).
- LOCK. It is offered, never set, and never will be by a re-runnable
  suite.
- Everything in table 99 (the low-power modes): there is no PWR driver
  in this stratum, so "comparator interrupts cause the device to exit
  Stop" is a sentence and not a measurement.
