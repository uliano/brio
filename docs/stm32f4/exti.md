# EXTI and SYSCFG (STM32F4)

Documents of record: RM0090 Rev 22, the EXTI ch. 12 and SYSCFG ch. 9 (the
vector tables are 62 and 63, one per part class); their twins RM0390 Rev 6
(ch. 10 and 8) and RM0383 Rev 4 (ch. 10 and 7), which differ in exactly one
thing - which peripheral wake-up lines the part has. Errata: no item of ES0206 Rev 24,
ES0298 Rev 8, ES0287 Rev 6 or ES0321 Rev 14 is filed against either block, and the one
item that is about them all the same is filed under the RTC - ES0206 2.9.3,
ES0298 2.10.3, ES0287 2.8.3, ES0321 2.11.3, every revision of all four parts - and is
quoted under "what the silicon does" below. Drivers: `stm32f4/exti.hpp` and
`stm32f4/syscfg.hpp`; the per-part line facts come from
`stm32f4/device_tables.hpp`. Bench suite: `test_stm32f4_exti`. Family
fixture `test/family_stm32f4/exti.cpp` plus four negatives under `brio
check stm32f4`.

## What the silicon does

**This is where the pin interrupts are.** GPIO on this family has no edge
sense, no interrupt and no flag: a pad's edges are the EXTI's, from the
multiplexer to the vector - the STM32G0's situation and the SAM C21's, and
the opposite of the AVR's, where every pin carries its own `PINnCTRL.ISC`.

**Twenty-three lines, or twenty-four.** Sixteen are the pin lines and the
rest are peripheral wake-ups: 16 the PVD, 17 the RTC alarm, 18 the USB OTG
FS wake-up, 19 the Ethernet wake-up, 20 the USB OTG HS wake-up, 21 the RTC
tamper and timestamp, 22 the RTC wake-up (12.2.5). **Which of them exist is
a per-part fact and the PERIPHERAL is what decides**: RM0390 has no line 19
(the F446 has no Ethernet MAC) and RM0383 neither 19 nor 20 (the F411 has
neither), and the reserve derives the set from the controllers' own
base-address macros rather than from a table. The EXTI's own bit macros are
no authority there - the pack spells `EXTI_IMR_MR18`, `MR19` and `MR20` on
every header, including parts with no USB at all - but above 22 they are
the only authority and they are used: a 24th line exists exactly on the
headers that declare `EXTI_IMR_MR23`, which are exactly the parts with an
LPTIM1. What is wired to it is the business of a reference manual this
project has not read, and no driver here names it.

**The line number is the pin number, and the port is the choice.** PA3, PB3
... PK3 all reach line 3, and `SYSCFG_EXTICR1`'s fourth nibble (codes 0..10
for ports A..K, 9.2.3) says which one does. So there is no pad-to-line
table to read, and no per-package gate either. The price is that **the
sixteen lines are a scarce resource shared by up to eleven ports**: PA5 and
PB5 cannot both raise interrupts, and the silicon has no arbitration - the
last EXTICR write takes the line and the pad that had it goes quiet with no
flag anywhere. The driver refuses that write (see "types and verbs").

**The multiplexer is another peripheral's register**, not the EXTI's as on
the STM32G0: SYSCFG's, on APB2, behind `RCC_APB2ENR.SYSCFGEN`, which is
CLEAR out of reset (measured). With that gate shut the block reads back
zero and drops every write in silence, so every verb of `Syscfg` - reads
included - opens it first.

**The lines are EDGE triggered and nothing else.** There is no level sense
(12.3.3's own note). Rising and falling are two independent bits in two
registers, and setting both is how a both-edges line is spelled. 12.3.3 and
12.3.4 differ in one detail worth knowing when a line is armed under a
moving pad: a rising edge that arrives *while EXTI_RTSR is being written*
does set the pending bit, a falling one during an FTSR write does not.

**One pending bit per line, for both edges.** `EXTI_PR` (12.3.6) is a
single rc_w1 register - unlike the STM32G0's RPR/FPR pair, a both-edges
line here does not say which edge arrived, and the pad's level at handler
time is the only clue.

**The pending bit is only set for an UNMASKED interrupt.** The chapter
never says so; figure 41 draws the interrupt mask between the edge detector
and the pending request register, and the bench agrees - with `EXTI_IMR`
clear, eight edges on an enabled trigger leave `PR` at zero and unmasking
afterwards does not resurrect them. **So a line cannot be watched without
being armed**; what replaces the "poll the flag with the interrupt off"
idiom is arming the EXTI's mask and leaving the NVIC line disabled. And a
line watched that way still sends its request to the NVIC, **which latches
it**: clearing `PR` does not clear the NVIC's own pending bit, so opening
the vector later delivers one call for an edge already acknowledged
(measured). `Nvic::clear_pending()` is the second half of that teardown.

**The edge detector needs the level to stand.** EXTI sits on APB2 and
samples the pad there; a pulse made of two adjacent `BSRR` stores - one
AHB cycle wide at 180 MHz - goes unseen (measured, none of eight on the
STM32F446 and on the STM32F429), and a pair of stores a few instructions
apart is MARGINAL: the same source registered on one part and not on the
other. A program that makes an edge for its own line holds the level for
at least one APB2 period; the suite holds it a microsecond.

**Every implemented line is configurable.** RTSR, FTSR, SWIER and PR all
carry bits 22:0, so every line has a trigger selection, a software trigger
and a pending bit: there is no "direct line" class as on the STM32G0, and
the peripheral wake-ups are edge-detected lines exactly like the pins - a
software trigger on line 22 raises the RTC wake-up vector with no RTC
involved (measured).

**A software trigger needs no pad, obeys the mask, and does not clear
itself.** `EXTI_SWIER` raises an edge on a line with no trigger selected at
all; but 12.3.5 makes the flag conditional on `EXTI_IMR` ("if interrupt are
enabled on line x ... writing '1' to SWIERx bit when it is set at '0' sets
the corresponding pending bit"), and the bit then **stands until the
line's PR bit is cleared**. Both halves measured: with the interrupt masked
a trigger raises nothing and leaves its SWIER bit set, and with it unmasked
the second trigger of a pair does nothing until the first is acknowledged.
A line is therefore cleared before it is armed, and `trigger()` is a
read-modify-write of one bit rather than the plain store the STM32G0's
self-clearing SWIER allows.

**Seven vectors for the pin lines, one per peripheral line.** Lines 0..4
have a vector each, 5..9 share EXTI9_5 and 10..15 share EXTI15_10 (table
62); every line above 15 reaches the vector of the peripheral wired to it -
PVD, RTC_Alarm, OTG_FS_WKUP, ETH_WKUP, OTG_HS_WKUP, TAMP_STAMP, RTC_WKUP -
which is why the wake-up vectors are declared on exactly the headers whose
part has the line. A handler on a shared vector is a dispatcher, and
because one pending register serves every line it must confine itself to
its own (measured: a handler saw another vector's flag and left it exactly
where it found it).

**The EXTI itself has no clock gate and no reset.** No device header of the
family declares an `EXTIEN` or an `EXTIRST`; only SYSCFG next door has one.
The edge detection is asynchronous, which is what makes an EXTI line a
wake-up source with the clocks stopped.

**An EXTI line returns the core from WFE with no interrupt at all.**
`EXTI_EMR` unmasks a CPU EVENT: no NVIC line, no handler and - 12.2.3 - no
pending bit to acknowledge. Measured against a masked control below. This
is a Sleep-mode statement (SLEEPDEEP is never written by this stratum);
Stop and Standby are the power chapter's.

**THE ERRATA ITEM THAT IS FILED SOMEWHERE ELSE.** "RTC interrupt can be
masked by another RTC interrupt" (ES0206 2.9.3, ES0298 2.10.3, ES0287
2.8.3, ES0321 2.11.3) is an RTC item whose mechanism is this peripheral's: **the effective
clear of an EXTI pending bit is delayed with respect to the store that asks
for it**, so a handler that checks a peripheral's own flags *before*
clearing the EXTI line can lose an event that arrives in between, and a
handler that clears last can be entered again for an edge already served.
The driver's answer is structural - `Exti::isr()` reads and clears at the
top, before the handler does anything with what it read.

**Compensation cell, memory map, PHY select** - the three switches that
share SYSCFG's block with the multiplexer. The I/O compensation cell (9.1)
is powered down by default and is *recommended*, not required, once an
output buffer is driven in the 50 or 100 MHz speed class: it holds the rise
and fall times steady and keeps the switching noise off the supply, and it
is legal from 2.4 V up. `SYSCFG_MEMRMP` says what is mapped at address 0 -
the BOOT pins' choice unless something overrode it - and the driver only
reads it, because moving the memory under a running program is a boot
decision and not a verb. `SYSCFG_PMC`'s `MII_RMII_SEL` picks the Ethernet
PHY interface and must be written while the MAC is in reset and before its
clocks are on (9.2.2), which is the Ethernet driver's sequence, not this
one's.

## Types and verbs

The driver owns the FABRIC - lines, triggers, the pending bits, the masks,
the multiplexer, the vectors - and NOT the vocabulary of what is wired to a
line above 15: a peripheral driver that owns a wake-up publishes its own
line number, and `Exti::implemented()` (run time) or `ExtiLine<n>` (compile
time) is how such a number is checked against the device.

- `ExtiSense` - `none` / `rising` / `falling` / `both` (there is no level);
  `exti_sense_has_rising/falling`.
- `Exti` - `gpio_lines`, `implemented_mask`, `regs()`, `implemented(line)`
  / `gpio(line)`, `irq(line)` / `vector_lines(irq)`; `sense(line,
  ExtiSense)` and `sense(line)`; `trigger(line)` / `triggered(line)`;
  `pending()` / `pending(line)` / `clear_lines(mask)` / `clear(line)`;
  `interrupt(line, on)` / `interrupt(line)` and `event(line, on)` /
  `event(line)`; `select(line, port)` (refusing) / `steal(line, port)`
  (not) / `selected(line)` / `in_use(line)`; `isr(lines)` /
  `served(fired, line)`; `release(line)`.
- `ExtInt<Pin>` - the pad's face: `line`, `mask`, `port`, `irq()`;
  `select()` / `steal()` / `selected()` / `claim(PinPull)`;
  `configure(ExtiSense)` / `sense()`; `arm(on)` / `armed()`; `event(on)` /
  `event()`; `trigger()` / `triggered()`; `pending()` / `clear()` /
  `served(fired)`; `release()`.
- `ExtiLine<n>` - a line named as a CONSTANT, which is what a peripheral
  wake-up always is: the same verbs minus the pad's, and a static_assert
  that refuses a line this part has not got.
- `exti_lines_distinct<Ints...>()` - the one-pin-per-line rule as a
  compile-time check on an application's OWN set of lines.
- `Syscfg` - `clock(on)` / `clock()`, `memory_map()` (`MemoryMap`),
  `exti_source(line, port)` / `exti_source(line)`,
  `compensation_cell(on)` / `compensation_cell()` /
  `compensation_ready()`, `has_phy_select()` / `phy_rmii(on)` /
  `phy_rmii()`.
- In the reserve (`stm32f4/device_tables.hpp`): `exti_gpio_lines`,
  `exti_port_code`, `exti_implemented_mask`, `exti_line_implemented`,
  `exti_line_irq`, `exti_vector_lines`, `syscfg_clock_mask`,
  `syscfg_phy_select_mask`.

**Two refusals worth naming.** `select()` fails when the line is IN USE by
another port - a sense selected, or the interrupt or the event unmasked -
because the silicon would let the write through and silence the pad that
had it; `steal()` is the same store without the question, for a program
reconfiguring lines it owns. A line merely POINTED at another port and
otherwise untouched is free, which is what keeps EXTICR's reset value (port
A everywhere) from being a claim by port A.

## How to use it

```cpp
using Button = brio::Pin<'C', 13>;
using ButtonInt = brio::ExtInt<Button>;          // line 13, port C
static_assert(brio::exti_lines_distinct<ButtonInt>());

ButtonInt::claim(brio::PinPull::up);             // input + pull + EXTICR
ButtonInt::configure(brio::ExtiSense::falling);
ButtonInt::clear();                              // before arming, never after
ButtonInt::arm(true);                            // EXTI_IMR - without it
                                                 // there is no flag at all
brio::Nvic::enable(ButtonInt::irq());            // EXTI15_10

extern "C" void EXTI15_10_IRQHandler() {
    const uint32_t fired = brio::Exti::isr(brio::Exti::vector_lines(EXTI15_10_IRQn));
    if (ButtonInt::served(fired)) {
        brio::post<Ui>(Pressed{});               // post() is ISR-safe
    }
}
```

A line WATCHED without a handler: arm the EXTI's mask, leave the NVIC line
disabled, read `pending()` - and when the watch ends, clear the NVIC's
pending bit as well as the EXTI's.

A peripheral wake-up, named as the constant it is:

```cpp
using RtcWake = brio::ExtiLine<22>;              // refused where there is none
RtcWake::configure(brio::ExtiSense::rising);
RtcWake::arm(true);
brio::Nvic::enable(RtcWake::irq());              // RTC_WKUP
```

A line that returns the core from `WFE` with no handler:
`ExtInt<Pin>::event(true)` instead of `arm(true)`, and nothing to clear
afterwards.

The compensation cell, for a program driving pads in the fast speed
classes:

```cpp
brio::Syscfg::compensation_cell(true);
while (!brio::Syscfg::compensation_ready()) { }
```

## Bench findings

`test_stm32f4_exti`, 12 letters in `z` (plus `p`, which waits for a press
and is not part of `z`), 65 verdicts, WIRELESS - on the Nucleo-F446RE
(DEV_ID 0x421, REV_ID 0x1000) at 180 MHz, and the same 65 on the
32F469IDISCOVERY (DEV_ID 0x434, LD1 on PG6 as the pad the program
drives, the blue button on PA0 reading 1 when pressed). Two stimuli, both inside the
chip: the software trigger, which needs no pad and reaches the peripheral
lines too, and **a pad the program drives itself** - the board's LED - since
a GPIO in output mode still feeds its input buffer (8.3.10). The pads
touched are the LED's and the user button's; the console (PA2/PA3) and the
SWD pads are avoided.

**The reset state.** IMR, EMR, RTSR, FTSR, SWIER and PR all come up at
zero - nothing is unmasked, triggered or pending behind a program's back,
which is not the STM32G0's story - and all four EXTICR registers come up at
zero, which is port A on every line. `RCC_APB2ENR.SYSCFGEN` comes up
CLEAR, and the driver's own verbs are what open it: a multiplexer write
issued with the gate shut lands correctly, because the verb opens the gate
before storing.

**The implemented set on this part is 0x77FFFF** - lines 0..22 without 19,
the Ethernet wake-up, which is exactly what RM0390 lists. Every one of the
five lines above 15 whose vector the suite binds (16, 17, 18, 21, 22)
reached ITS OWN vector on a software trigger, with no peripheral
configured; line 20 (USB OTG HS) flags like any other; and line 19 refuses
the sense, the trigger, the mask and the release, and has no vector.

**A pad in output mode feeds its own line.** Eight program-made rising
edges: 8 flags on a rising sense, 0 on a falling one; the eight falling
edges: 0 and 8; both edges: 8 and 8; no trigger selected: 0 and 0. **Analog
mode is the one state that blinds a line** - with the input buffer off, the
same sixteen stores produce no edge at all.

**The pending bit needs the mask.** With `IMR` clear, eight edges left `PR`
at zero and unmasking afterwards left it at zero. With `IMR` set and the
NVIC line disabled the flag stands and no handler runs - and the NVIC
latched the request all the same: after clearing the EXTI's flag, enabling
the vector still delivered exactly one call. Clearing both halves delivers
none.

**The software trigger, in numbers.** From the SWIER store to the handler's
first instruction: **71..96 cycles over sixteen rounds, mean 72 - 400 ns at
180 MHz**, which is the exception entry and not the peripheral. Five
triggers in a row reached the vector five times; with the interrupt masked
a trigger raised no flag and left its SWIER bit standing.

**The grouped vectors.** Eight triggers alternating between line 5 (port A)
and line 6 (port B) produced eight calls of EXTI9_5, four attributed to
each line; two lines raised inside one critical section produced ONE call
with both lines in the mask (0x60). With line 10 left standing and
EXTI15_10 shut at the NVIC, the EXTI9_5 handler saw the foreign flag once
and left it exactly where it found it, serving only its own line - and the
other vector took it the moment its NVIC line opened.

**The CPU event out of WFE**: **0 us with `EMR` set against 999 us for the
masked control**, which is the next SysTick, and no pending bit either way.
Two traps stand in the way of that measurement, both the STM32G0's: an
exception entry sets this core's event register just as SEV does (so the
order is align to the tick, clear the register with SEV + WFE, make the
edge, sleep), and every byte of a verdict line is a USART interrupt, which
returns a WFE too (so the console is drained first and the printing comes
after).

**The pending bit's clear.** A zero written over a standing flag leaves it,
and so does a one written at every other line: `PR` is rc_w1 and the driver
never read-modify-writes it. The store and the read-back that follows it
cost 80 cycles together, and the read already sees the flag gone. **A
handler that clears its line as its LAST store was entered once, not
twice** - the delay the errata item describes did not outlive the exception
return on this part. That is a measurement and not a licence: the driver
still clears at the top, which is what the errata asks for and costs
nothing.

**The compensation cell** reported READY 38 to 39 polls of `CMPCR` after
being powered up.

**The board's button** is on line 13 of port C, vector 40, and reads HIGH
at rest against no internal pull: the board holds it up, so a press is a
FALLING edge. Armed for 50 ms with nobody pressing, the line raised
nothing. No verdict in `z` depends on a press - it cannot be staged from
here - which is why the letter that waits five seconds for one, `p`, is
outside `z` and fails when nobody is at the desk.

## Not covered yet

Driver gaps:

- `SYSCFG_PMC`'s `ADCxDC2` bits: they belong to a peripheral chapter with
  no driver in this stratum, and they are struct members on some headers
  only - born with their first user. (`SYSCFG_CFGR`, the other such
  register, was one of these until the Fast-mode Plus I2C arrived: its two
  pad-drive bits are `Syscfg::fast_mode_plus()`, and
  [fmpi2c.md](fmpi2c.md) is what uses them.)
- `SYSCFG_MEMRMP` is read and never written: moving what lives at address 0
  under a running program is a boot decision the BOOT pins already made,
  and the swap bits the bigger parts add there (the FMC/FSMC bank swap, the
  flash bank swap) belong to those chapters.
- The peripheral bit-band alias (RM0090 2.3.3), which would make every
  one-bit verb here atomic against a handler instead of a
  read-modify-write: declined for now because the configuring verbs are
  setup-time by contract, as everywhere else in this stratum, and the ISR
  body's only write is to `PR`, which is rc_w1 and needs no read. A
  `trigger()` raised from two contexts is the one call that would gain by
  it.
- Wake-up from **Stop** and **Standby** through an EXTI line: the lines and
  their masks are here, but arming a deep sleep is the power chapter's, and
  there is no `SleepSite` in this stratum yet. What an EXTI line contributes
  there is exactly its IMR or EMR bit, and what the clocks cost coming back
  is that chapter's to measure.
- The lines above 15 have no OWNERS yet: every one of them belongs to a
  peripheral - PVD to PWR, three to the RTC, two or three to the USB
  controllers, one to the Ethernet MAC - and each of those drivers will
  publish its own line number the way this family's serial instances
  publish their vectors. Until then an application spells the number
  itself, and `ExtiLine<n>` is what checks it.

Implemented, not bench-verified:

- The parts with more than eleven ports and the codes above 7 (`ExtInt` on
  ports I, J and K): compile-checked on every header, and the port code is
  arithmetic on the letter, so a bonded pad of those ports would carry
  straight over - it needs one of the big packages.
- The 24th line the F410 and F413/F423 headers declare: no board here has
  one of those parts, and what is wired to it is not in a manual on this
  desk.
- `Syscfg::phy_rmii()` on a part with an Ethernet MAC: it answers false and
  writes nothing here (the F446 has no MAC). It needs a part that has one -
  and, to mean anything, an Ethernet driver.
- The Ethernet and USB OTG HS wake-up lines (19 and 20) as INTERRUPTS: line
  20 is flagged and cleared here but the suite binds no handler for either
  vector, both being the parts' and not this board's.
