# EXTI (STM32G0)

Documents of record: RM0444 Rev 6, EXTI ch. 13; the vector table is
table 61 (12.3), the low-power modes chapter 4, the GPIO chapter 7
(where the pads come from). Errata ES0548 Rev 3 on silicon revision Z:
**no item touches the EXTI**; the adjacent one is 2.3.1, GPIO
configuration after a Standby wake-up, and it is revision A only
([pwr.md](pwr.md), whose suite is what enters Standby). Driver:
`stm32g0/exti.hpp`; the per-part line facts come from
`stm32g0/device_tables.hpp`. Bench suite: `test_stm32_exti`. Family
fixture
`test/family_stm32g0/exti.cpp` plus three negatives under
`brio check stm32g0`.

## What the silicon does

**This is where the pin interrupts are.** GPIO on this family has no
edge sense, no interrupt and no flag: a pad's edges are the EXTI's from
the multiplexer to the vector - the SAM C21's EIC situation, and the
opposite of the AVR's, where every pin carries its own `PINnCTRL.ISC`.

**The line number IS the pin number, and the port is the choice.** PA3,
PB3, PC3, PD3, PE3 and PF3 all reach line 3, and `EXTI_EXTICR1`'s third
8-bit field (codes 0..5 for ports A..F, 13.5.11) says which one does.
So there is no pad-to-line table to read - and no per-package gate
either, unlike the SAM's irregular map. The price is that **the sixteen
lines are a scarce resource shared by six ports**: PA3 and PB3 cannot
both raise interrupts, the last write to EXTICR simply takes the line,
and nothing in the silicon warns. Measured: with line 0 pointed at port
A, PA0's edge is the line's and PB0's edge is invisible; pointing it at
port B swaps the two answers exactly, while the line's sense, mask and
pending bits are untouched by the move.

**The lines are EDGE triggered, and nothing else.** There is no level
sense (13.5.1's own note). Rising and falling are two independent bits
in two registers, and setting both is how a both-edges line is spelled.

**Rising and falling have SEPARATE pending bits**, `EXTI_RPR1` and
`EXTI_FPR1` (13.5.4, 13.5.5), both write-1-to-clear. A both-edges line
therefore tells a handler WHICH edge arrived without reading the pad -
something neither the AVR's nor the SAM's single flag per line can do.
Measured on 8 up and 8 down edges: a rising line flags 8/0, a falling
line 0/8, a both line 8/8, a line with no trigger 0/0.

**The pending bit is only set for an UNMASKED interrupt.** 13.3.1 says
it in those words and 13.4 repeats it, and the bench confirms it: with
`EXTI_IMR` clear, eight edges on an enabled trigger leave `RPR1` and
`FPR1` at zero, and unmasking afterwards does not resurrect them. **So
a line cannot be polled without arming it** - the "flag stands while
the interrupt is masked" idiom that both other brio targets offer does
not exist here. What replaces it is arming the EXTI mask and leaving
the NVIC line disabled: the flag then stands, no handler runs, and that
is how the wireless letters of the suite count edges.

**A software trigger is a rising edge and needs no pad.** `EXTI_SWIER`
raises one "independently of EXTI_RTSR and EXTI_FTSR" (13.5.3): five
writes gave five rising flags, no falling flag, five handler calls
through the real vector on a line with NO trigger edge selected at all.
The bit clears itself and reads back zero. With the interrupt masked it
leaves no pending bit either - 13.3.1's rule is about the pending
register and not about where the edge came from.

**Three vectors serve the sixteen lines**: EXTI0_1, EXTI2_3, EXTI4_15
(table 61). Every handler is therefore a dispatcher. Measured with four
lines armed at once (0, 3, 7 and 8, both edges, four rounds): every
edge reached a handler, line 0 was served by EXTI0_1 only, line 3 by
EXTI2_3 only, lines 7 and 8 by EXTI4_15, and each handler ran exactly
as many times as its own lines fired.

**One pending register serves every line, so a handler must confine
itself.** Staged deliberately: line 7 left pending with EXTI4_15 shut
at the NVIC, then EXTI0_1 made to run. Its body SAW line 7's bit
(`0x80` outside its own lines) and left it exactly where it found it,
because `Exti::isr()` is handed the mask of the lines its vector
answers for. A shared register does not make a shared handler.

**An EXTI line can return the core from WFE with no interrupt at all.**
`EXTI_EMR` unmasks a CPU EVENT: no NVIC line, no handler, and - 13.4 -
no pending bit to acknowledge. Measured with the interrupt masked and
the event unmasked: **an edge before the WFE makes it return in 0 us,
where the same edge with the event masked leaves the WFE waiting 798
us for the next SysTick**, and the line's pending bits are zero either
way. This is a Sleep-mode measurement (SLEEPDEEP never written); Stop
and Standby are not measured here.

**The block has no clock gate and no reset.** It is on the AHB and runs
on hclk; no device header of the family declares an `EXTIEN` or an
`EXTIRST`, so nothing has to be turned on before a line is used - the
opposite of GPIO, whose port clock every configuring verb in
`pin.hpp` must open first. The edge detection itself is asynchronous
(13.3.1), which is what makes an EXTI line a wake-up source.

**Configurable and DIRECT lines are two different things** (13.3,
table 64). A configurable line has a trigger selection, a software
trigger and a pending bit; a **direct** line has none of the three -
its flag lives in the peripheral, its interrupt reaches the CPU through
the peripheral's own vector, and all the EXTI does for it is wake the
system. IMR/EMR are the only registers that touch one, which is why
every mask verb in the driver is a read-modify-write of ONE bit.

**And the reset value of IMR1 proves it.** 13.5.12 states the rule -
"the reset value is set such as to, by default, enable interrupt from
direct lines, and disable interrupt from configurable lines" - and then
prints **0xFFF8 0000**, which would leave line 20 (COMP3) unmasked
although line 20 is a CONFIGURABLE line on this part. The silicon
follows the RULE and not the number: **IMR1 comes up at 0xFFE8 0000**,
which is exactly `implemented & ~configurable` computed from the device
header's own masks, bit for bit. IMR2 comes up at 0x1B, where the
printed value and the rule agree. EMR1, RTSR1 and EXTICR1 come up at
zero.

**Which lines exist is per part, and the device header says so.**
`EXTI_IMR1_IM_Msk` is a per-variant mask and not a blanket one -
0xF2A9FFFF on the G031, 0xFEAFFFFF on the G071, 0xFFFFFFFF on the G0B1
- so it is the authority on which of lines 0..31 a part implements;
`EXTI_IMR2_IM_Msk` is its twin for 32..36 and does not exist where
there is no second group. The configurable set comes from the RTSR
masks the same way (0x0001FFFF, 0x0007FFFF and 0x0017FFFF respectively,
plus line 34 on the G0B1). The second register group is a STRUCT MEMBER
question - `RTSR2`/`FTSR2`/`SWIER2`/`RPR2`/`FPR2` are members on the
G0B1/G0C1 only, `IMR2`/`EMR2` from the G071 class up - so the reserve
exports them as pointers, null where the register does not exist, and
the driver names no register a smaller part has not got.

**Table 65 for the bench part, as documentation and not as vocabulary**
(see "Types and verbs" for why the driver does not own it): line 16
PVD (configurable), 17/18 COMP1/COMP2 (configurable), 19 RTC (direct),
20 COMP3 (configurable), 21 TAMP, 22 I2C2, 23 I2C1, 24 USART3, 25
USART1, 26 USART2, 27 CEC, 28 LPUART1, 29 LPTIM1, 30 LPTIM2, 31
LSE_CSS, 32 UCPD1, 33 UCPD2 (all direct), 34 VDDIO2 monitoring
(configurable), 35 LPUART2, 36 USB (direct). **This list is per part**:
the G031 has neither 20 nor 22 nor 24, the G071 neither 20 nor 22.

**Lines 29 and 30 have an owner**: `stm32g0/lptim.hpp` publishes them as
`Lptim<n>::exti_line` and opens them with `Lptim<n>::wake_line(true)` -
the numbers are the MANUAL'S, no header of this pack spells them, and
they are what lets a low-power timer's compare match bring the core out
of Stop ([lptim.md](lptim.md), [pwr.md](pwr.md)).

## Types and verbs

The driver owns the FABRIC and not the vocabulary of what is wired to a
line above 15: a peripheral that owns a wake-up publishes its own line
number, and `exti_line_implemented()` / `exti_line_configurable()` are
how such a number is checked against the device header. What lives here
is what is uniform: the sixteen GPIO lines.

- `ExtiSense` - `none` / `rising` / `falling` / `both` (there is no
  level); `exti_sense_has_rising/falling`.
- `ExtiPending {rising, falling}` - one vector's worth of pending bits,
  with `lines()` and `any()`.
- `Exti` - `gpio_lines`, `regs()`, `implemented(line)` /
  `configurable(line)` / `gpio(line)`, `irq(line)` /
  `vector_lines(irq)`; `sense(line, ExtiSense)` and `sense(line)`;
  `trigger(line)` (SWIER); `rising_pending()` / `falling_pending()` /
  `clear_rising(mask)` / `clear_falling(mask)` and their per-line
  twins plus `pending(line)` / `clear(line)`; `interrupt(line, on)` /
  `interrupt(line)` and `event(line, on)` / `event(line)`;
  `select(line, port)` / `selected(line)`; `isr(lines)`;
  `release(line)`.
- `ExtInt<Pin>` - `line`, `mask`, `port`, `irq()`; `select()` /
  `selected()` / `claim(PinPull)`; `configure(ExtiSense)` / `sense()`;
  `arm(on)` / `armed()`; `event(on)` / `event()`; `trigger()`;
  `rising_pending()` / `falling_pending()` / `pending()` / `clear()`;
  `release()`.
- `exti_lines_distinct<Ints...>()` - the one-pin-per-line rule as a
  compile-time check on an application's OWN set of lines.
- In the reserve (`stm32g0/device_tables.hpp`): `exti_gpio_lines`,
  `exti_implemented_mask1/2`, `exti_configurable_mask1/2`,
  `exti_line_implemented`, `exti_line_configurable`, `exti_port_code`,
  the seven group-2 register pointers, `exti_gpio_irq`,
  `exti_vector_lines`.

## How to use it

```cpp
using Button = brio::Pin<'C', 13>;
using ButtonInt = brio::ExtInt<Button>;          // line 13, port C
static_assert(brio::exti_lines_distinct<ButtonInt>());

ButtonInt::claim(brio::PinPull::up);             // input + pull + EXTICR
ButtonInt::configure(brio::ExtiSense::falling);
ButtonInt::arm(true);                            // EXTI_IMR - without it
                                                 // there is no flag at all
brio::Nvic::enable(ButtonInt::irq());            // EXTI4_15

extern "C" void EXTI4_15_IRQHandler() {
    const brio::ExtiPending p =
        brio::Exti::isr(brio::Exti::vector_lines(EXTI4_15_IRQn));
    if (p.falling & ButtonInt::mask) {
        brio::post<Ui>(Pressed{});               // post() is ISR-safe
    }
}
```

A line watched without an interrupt (the polling recipe): arm the
EXTI's mask, leave the NVIC line disabled, read `pending()`.

A line that returns the core from `WFE` with no handler:
`Exti::event(line, true)` instead of `arm(true)`, and nothing to clear
afterwards.

## Bench findings

`test_stm32_exti`, 9 letters, 89 verdicts, WIRELESS. Pads PA0/PB0 (line
0), PB3 (line 3), PB7 (line 7), PA8 (line 8), PC13 (the user button,
read only); the console (PA2/PA3), LD4 (PA5), SWD (PA13/PA14), the LSE
pads (PC14/PC15) and the HSE pads (PF0/PF1) are all avoided.

**The stimulus, measured before anything rests on it.** All five pads
follow their own internal pull (input mode, PUPDR up then down, read
IDR), and PA0 and PB0 are independent of each other. And the fact that
makes a wireless bench easy on this family: **the EXTI sees a pad its
owner is DRIVING**. The multiplexer selects a PORT and not a pin
function, and the input buffer stays live in output mode (7.3.1), so a
pad in OUTPUT mode driving itself high is a clean edge on its line -
where the SAM C21's PMUXEN takes the pad away from PORT's output driver
entirely and only the pull remains. Both techniques are exact: 16 of 16
driven edges, 16 of 16 pull-walked edges, no edge ever double-counted
by an unfiltered detector. **Analog mode is the one that blinds a
line**: with the input buffer off, the pad's own pull moves nothing the
EXTI can see.

**The reset value of IMR1** (see above): 0xFFE80000, not the 0xFFF80000
13.5.12 prints - the manual's own rule beats the manual's own number,
and the device header's two masks predict the silicon bit for bit.

**The CPU event out of WFE**: 0 us against 798 us for the masked
control, with no pending bit set either way.

**Two measurement traps**, both worth knowing:

- **An exception entry sets the core's event register**, not just SEV
  and the EXTI - so a loop that waits for a tick edge (and therefore
  ends by returning from `SysTick_Handler`) leaves the register SET,
  and a WFE right after it returns at once whatever the EXTI did. The
  order that measures anything is: align to the tick, THEN clear the
  event register with SEV + WFE, THEN make the edge, THEN sleep.
- **A verdict line is milliseconds of console, and every byte of it is
  a USART interrupt** - an interrupt pending in the NVIC returns WFE
  too (4.2.2). With a print between the arming and the sleep, both legs
  measure zero: the measurement computes its verdicts first, drains the
  console, sleeps, and prints afterwards.

**The board's own button.** PC13 reads HIGH against an internal
pull-down, against an internal pull-up and with no pull at all: the
Nucleo holds it high with an external pull-up stronger than the
internal pull-down, so **a press is a FALLING edge** on line 13. Armed
for 50 ms with nobody pressing, the line raised nothing. No verdict
depends on a press - it cannot be staged from here - but the letter
prints the level, so running `u` with the button held says what it
should.

## On the STM32G071RB

`test_stm32_exti` runs on the Nucleo-G071RB (DEV_ID 0x460, REV_ID
0x2000), where the controller has FEWER LINES: `EXTI_IMR1_IM_Msk` is
0xFEAFFFFF against the G0B1's 0xFFFFFFFF - lines 20 (COMP3), 22 (an I2C2
wake) and 24 are not implemented there - and the second register group
has its MASK registers (IMR2, EMR2) but NO TRIGGER registers at all, no
line above 31 being configurable. The reserve answers the second group
with POINTERS that are null when the register is absent
(`exti_rtsr2()`, `exti_ftsr2()`, `exti_swier2()`, `exti_rpr2()`,
`exti_fpr2()`), which is why the suite judges their presence against
`exti_configurable_mask2` rather than dereferencing them.

**THE TWO DIES TAKE OPPOSITE SIDES OF A MANUAL THAT DISAGREES WITH
ITSELF.** 13.5.12 states the reset value as a RULE in words - "enable
interrupt from direct lines, and disable interrupt from configurable
lines" - and then PRINTS 0xFFF8 0000 beside it. The G0B1RE obeys the
rule, and its implemented mask being all ones the two readings coincide.
The G071RB holds **the printed number**: IMR1 comes up 0xFFF80000 with
its implemented mask 0xFEAFFFFF, so bits 20, 22 and 24 - lines it does
not implement - read back SET. The verdict is therefore over the
IMPLEMENTED bits, where both dies agree bit for bit, and the residue is
printed. An unimplemented mask bit does nothing either way, which is why
this is a finding about the document and not about a program.

## On the STM32G031K8

`test_stm32_exti` scores **85 of 89** on the Nucleo-G031K8 (DEV_ID 0x466,
REV_ID 0x1003). The four not claimed are one leg and one letter, each
skipped by name: IMR2's reset value, because **this part has no second
register group at all** (`exti_implemented_mask2` is zero, the reserve's
`exti_imr2()`/`exti_emr2()` pointers are null beside the trigger ones -
so the group's presence is now judged as the EQUIVALENCE "mask registers
exactly where there are lines above 31"); and letter `u`, the user
button, because **the LQFP32 does not bond PC13** (DS12992 table 12) and
a Nucleo-32 has no button - an unbonded pad's level is not a reading, so
line 13 is left alone rather than configured.

**A THIRD IMR1 RESET VALUE.** This die comes up with IMR1 =
**0xFFFE0000** where the G071 holds 13.5.12's printed 0xFFF80000 and the
G0B1 the rule that section states in words. Over the lines the part
IMPLEMENTS all three agree exactly - here 0xF2A80000 of a 0xF2A9FFFF
mask, which is the direct set - and the residue 0x0D560000 sits above the
implemented mask and means nothing. The verdict is the rule over the
implemented bits; what a die holds above them is printed beside it.

**PA8 IS NOT UCPD1_CC1 HERE**, this part having no UCPD at all: the
dead-battery Rd is absent, `ucpd_dead_battery()` REFUSES instead of
writing a strobe bit that does not exist, and letter `b` claims the pad
follows its own pull AND that the verb answers exactly
`ucpd_present(1)` - a reserve fact probed on SYSCFG's own strobe bit.

## Not covered yet

Driver gaps: the direct lines whose peripheral this stratum has not
built - CEC, UCPD, USB - so their line numbers are published by nobody.
The lines that DO have an owner are the RTC's 19 and TAMP's 21
([rtc.md](rtc.md)), the comparators' 17/18/20 ([comp.md](comp.md)), the
USARTs' 24/25/26 and the LPUARTs' 28/35 ([usart.md](usart.md),
[lpuart.md](lpuart.md)), the LPTIMs' 29/30 ([lptim.md](lptim.md)) and
now the I2Cs' 22 and 23 ([i2c.md](i2c.md), where `i2c_exti_line()`
publishes them and only for the instances table 165 lets wake at all:
I2C1 always, I2C2 on the G0B1/G0C1 class, I2C3 never). THE DEVICE
HEADER'S OWN COMMENT DISAGREES about those two and table 65 wins: CMSIS
annotates the shared vector as "I2C2, I2C3 Interrupt (combined with EXTI
24 and EXTI 22)", but line 24 is USART3's wake and I2C3 has no wake at
all.
Still open: the configurable non-GPIO line VDDIO2 34, reachable through
`Exti` today but with no driver to publish its number or its vector
(PVD 16 is published by `pwr.hpp` as `pvd_exti_line`, and its crossing
is not stageable on a board whose VDD is the probe's -
[pwr.md](pwr.md)); and `SEVONPEND`, which changes what returns a WFE
and belongs to a kernel pass rather than to this chapter.

Wake-up from **Stop** through a direct line is covered: an LPTIM
compare match leaves both Stop 0 and Stop 1 through line 29, measured in
`test_stm32_lptim` letter g, an RTC alarm does the same through line 19
([pwr.md](pwr.md)), a USART's WUF leaves Stop 0 through line 26 and an
LPUART's leaves Stop 1 through line 28 ([usart.md](usart.md)). What a
direct line contributes there is exactly its IMR bit - it has no edge
selection and no pending bit of its own, the peripheral's own flag being
the pending state.

Implemented, not bench-verified: the second register group's own verbs
(line 34's trigger, pending and software trigger - `VDDIO2` monitoring
has no stimulus without a supply change); `ExtInt` on ports D, E and F
(compile-checked on every header; letter b's pull-walk on a bonded
port-D pad would carry straight over, and the LQFP64 bonds PD0..PD6);
`Exti::release()` on a direct line (every release a suite spends is on a
GPIO or a comparator line).

Stated, not enforced: **one pin per line**. `exti_lines_distinct<>()`
checks an application's declared set, and nothing makes an application
declare one - the same kind of claim as an AF number, which no header
of this family can check either. Errata ES0548 2.3.1 (a pad's
configuration after a Standby wake-up) is stated on the driver for the
revision it names, which is not this silicon's.
