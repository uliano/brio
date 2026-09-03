# Comparators (STM32G0)

> **PROVISIONAL.** All of chapter 18's register surface is implemented -
> the three input multiplexers, the hysteresis and the two speeds, the
> polarity, the blanking sources, the window mode and its output
> selector, the lock and the EXTI line each comparator publishes - and
> everything a bench with no wires can reach is verified. What CANNOT be
> reached here is an analog THRESHOLD: a comparator's non-inverting
> input is a pad and only a pad, and this family disconnects a pad's own
> pull the moment it goes analog, so no source inside the chip can hold
> that input at a chosen voltage. The consequences are stated where they
> bite and listed in "Not covered yet".

Documents of record: RM0444 Rev 6 ch. 18, with the EXTI line table
13.5.1, the vector table 12.3 (table 61) and TIM1_TISEL / TIM1_AF1
(21.4.27, 21.4.28); DS13560 Rev 5 table 12 (the input pads) and table 68
(offset, hysteresis, propagation delay); errata ES0548 Rev 3, which has
**no item touching the comparators**. Driver: `stm32g0/comp.hpp`; the
per-part presence and the EXTI line numbers come from
`stm32g0/device_tables.hpp`. Bench suite: `test_stm32_analog` letter i
(15 verdicts), shared with the ADC, the DAC and the reference buffer.
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

**COMP3 answers.** Its CSR is reachable through the SYSCFG gate and its
INMSEL reads back what was written; its three plus pads are left alone
on this desk and no verdict about a signal is offered on a comparator
this suite never gives one.

**The lock is clear and stays clear** on all three, which is what a
suite that must be re-runnable can say about a bit whose only exit is a
reset.

## Not covered yet

**Implemented but not bench-verified:**

- Every analog THRESHOLD question: the offset (DS13560's +/- 5 mV
  typical), the three hysteresis levels, the two propagation delays, and
  the window's INSIDE state. All four want the plus input held at a
  chosen voltage, which needs one wire from PA4 to a comparator input
  pad - and this campaign is wireless by construction.
- WINOUT, which did nothing measurable (above) and is recorded as such.
- The DAC as a threshold ON SILICON. `CompNegative::dac_channel1/2` is
  configured, validated against `dac_present()` and refused where there
  is no DAC; with no analog signal on a plus input there is nothing to
  compare it to yet.
- The comparator output ON A PAD. It is an alternate function
  (DS13560's AF tables) and this suite reaches the output through the
  EXTI line and TIM1's TI1 instead, both of which need no pin.
- The four blanking sources other than TIM1_OC4.
- COMP2's and COMP3's own plus pads, and COMP3 entirely as a signal
  path.
- LOCK. It is offered, never set, and never will be by a re-runnable
  suite.
- Everything in table 99 (the low-power modes): there is no PWR driver
  in this stratum, so "comparator interrupts cause the device to exit
  Stop" is a sentence and not a measurement.
