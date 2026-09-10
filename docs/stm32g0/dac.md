# DAC (STM32G0)

Documents of record: RM0444 Rev 6 ch. 16, with the DMAMUX request table
55 and the vector table 12.3 (table 61); DS13560 Rev 5 table 12 (the
pads) and table 64 (the DAC's electrical characteristics); errata
ES0548 Rev 3, which has **no item touching the DAC** on either silicon
revision. Driver: `stm32g0/dac.hpp`; the reference vocabulary is
`stm32g0/vref.hpp`'s. Bench suite: `test_stm32_analog` (shared with the
ADC, the reference buffer and the comparators). Family fixture
`test/family_stm32g0/dac.cpp` plus one negative under
`brio check stm32g0`.

## What the silicon does

**Two 12-bit channels, and on the G031/G041 - and on every x0
value-line part - there are none at all.** 16.3's own table says so and
the device headers agree by declaring no `DAC1_BASE` there, so `Dac`
does not exist on those parts (the driver compiles its register-facing
half only where the header declares the block; the trigger, mode and
wave vocabulary is every part's) and is a MONOSTATE on the others
(there is one DAC wherever there is one).
DAC1_OUT1 is PA4 and DAC1_OUT2 is PA5 - which on a Nucleo-64 is LD4.

**THE DAC REACHES THE ADC THROUGH THE PAD AND NOTHING ELSE.** 16.4.2
offers an "internal pin connection to on-chip peripherals" and
DAC_MCR.MODEx (16.7.16) spells eight combinations of pad, on-chip path
and buffer - but the ADC's own connectivity figure (15, figure 36) lists
nineteen inputs and the DAC is not among them. On this family the
internal path goes to the COMPARATORS. So a DAC-to-ADC measurement here
is a zero-length wire through one bond pad (PA4 is DAC1_OUT1 and ADC_IN4
at once), and a DAC-to-comparator one needs no pad whatever. Neither
chapter says this; it is the shape of every wireless analog experiment
on this part.

**Channels are 0-based in this driver and 1-based in the manual.**
`ch = 0` is DAC_CH1 / DAC1_OUT1 / PA4 and `ch = 1` is DAC_CH2 /
DAC1_OUT2 / PA5. Every channel verb of this stratum counts from zero
(`stm32g0/tim.hpp` says so for the timers) and one file counting
differently would be worse than one offset stated loudly.

**DAC_DHR is not DAC_DOR.** A write lands in the holding register and
reaches the output one dac_pclk cycle later with no trigger, or THREE
cycles after the trigger with one (16.4.5); DOR cannot be written at
all. `code(ch)` reads what was asked for and `output(ch)` what the
converter is producing, so a caller that confuses them measures its own
write.

**A trigger is an EDGE and the first datum must precede it.** 16.4.8:
"the very first data has to be written to the DAC_DHRx before the first
trigger event occurs" - which is why a DMA-fed stream starts by priming
the holding register, and why DMAUDRx exists. The underrun flag is
write-1-to-clear and the chapter's own recovery is to clear it, stop the
DMA and re-initialize both: this driver reports and the owner decides,
because only the owner knows what its stream was.

**The wave generators need a trigger.** WAVEx is "only used if TENx = 1"
(16.7.1), so `dac_channel_config_valid()` refuses noise or triangle
without one rather than quietly doing nothing. MAMPx is 2^(code+1) - 1
in both its readings (an LFSR mask and a triangle amplitude),
saturating at 4095 above code 11.

**The trigger table has holes.** Table 85 assigns dac_chx_trg1, 2, 3,
5, 6, 8, 11, 12 and 13 (TIM1/TIM2/TIM3/TIM6/TIM7/TIM15 TRGO, the two
LPTIM outputs and EXTI9) and leaves trg4, 7, 9, 10, 14 and 15 carrying
nothing. `DacTrigger` spells only what exists and `dac_trigger_valid()`
refuses the rest, so a code that would select a dead line is a false and
not a silent stop.

## The types and the verbs

`stm32g0/dac.hpp` is the reference. `DacChannelConfig` +
`dac_channel_config_valid()` carry the whole per-channel configuration
and the chapter's refusals; `Dac::configure(ch, cfg)` writes MCR and CR
with the channel's ENx and CENx down first, because 16.7.16 makes MODEx
and TSELx writable only then and a write that lands nowhere is the trap
that chapter warns about. Data goes in through `write` (12-bit right),
`write_left`, `write8` or `write_dual`; `trigger(ch)` is SWTRIGx;
`sample_hold_times()` writes the three low-power-domain registers and
`busy(ch)` is the BWSTx that stands until they land; `trim`/`set_trim`/
`calibration_mode`/`calibration_flag` are 16.4.12's user offset
procedure, with `calibration_mode()` refusing an unbuffered mode where
the chapter says it has no effect. The two DMAMUX request ids (table 55
rows 8 and 9) and the four data-register addresses are published for a
`stm32g0/dma.hpp` engine's `arm()`.

```cpp
brio::Dac::init();
brio::Dac::claim_pad<brio::Pin<'A', 4>>();          // DAC1_OUT1, analog mode
brio::Dac::configure(0, {.mode = brio::DacMode::pin_and_internal_buffered});
brio::Dac::enable(0, true);
brio::Dac::write(0, brio::dac_code(1650, brio::Dac::steps, vdda_mv));
```

## Bench findings

Measured by `test_stm32_analog` on an STM32G0B1RE at 3.3 V (VDDA
3310 mV, measured through VREFINT), silicon revision Z.

**The zero-length wire is straight.** DAC codes 0..4095 read back
through ADC_IN4 on the same pad:

| code | 0 | 512 | 1024 | 1536 | 2048 | 2560 | 3072 | 3584 | 4095 |
|---|---|---|---|---|---|---|---|---|---|
| ADC counts | 57 | 503 | 1014 | 1528 | 2039 | 2553 | 3063 | 3575 | 4053 |

Monotonic across the whole range, and the **worst residual from the
end-to-end line is 2 counts of 4096** - which is a DAC, a bond pad and
an ADC in series, reported as one number because nothing on this bench
can split them.

**THE OUTPUT BUFFER CANNOT REACH THE RAILS, and turning it off says by
how much.** Code 0 and code 4095 measure **46 mV and 3276 mV buffered**,
and **3 mV and 3308 mV unbuffered** against a 3312 mV supply. So the
buffer costs about 43 mV at the bottom and 32 mV at the top, the pad and
the ADC are not what is limiting it, and the unbuffered mode really does
span nearly the whole reference - at the price of driving nothing.

**The three data formats are PLACEMENTS of one 12-bit datum** (16.4.4):
DHR12R1 = 2048, DHR12L1 = 0x8000 and DHR8R1 = 128 all put 2048 in DOR.

**LD4 is not a load the DAC can feel.** Both channels run at once, one
on the free PA4 and one on PA5 = LD4, and the two agree to **5 mV at
every code** across the range - the worst difference is at code 1024,
not at the top where a conducting LED would droop the buffer. PA5 also
follows a 40 kohm internal pull between the rails exactly as the free
pads do. So whatever drives LD4 on this board is high-impedance seen
from the pin, and the obvious guess - a diode and a series resistor to
ground - is wrong. A DAC-driven brightness works either way, and it is
the one thing in this suite a human can see.

**One trigger, both converters, no CPU:** the chain is described in
[adc.md](adc.md)'s bench findings - TIM6's TRGO starting the DAC and the
ADC on the same edge, a `DmaLoopEngine` playing a 16-entry table into
DHR12R1 and a `DmaPingPongEngine` draining ADC_DR, with zero samples off
the table over six blocks and every seam stepping by exactly 8. On the
DAC side, **the launch block is out of phase by construction**, because
16.4.8's rule makes the first datum the CPU's and not the table's.

### The wave generators, the calibration and sample-and-hold (letter `o`)

**Both generators run, driven ONE STEP AT A TIME.** 16.4.9 and 16.4.10
want a trigger, and the SOFTWARE trigger makes each step the program's
own - so a triangle is measured as a sequence and not as a spectrum.
At MAMP 5 (amplitude 63) `DAC_DOR` climbs exactly 63 codes from the
holding register and comes back, **and it turns at the top**: the first
fall is the step after the amplitude is reached, which is what makes the
period twice the amplitude and not twice the amplitude plus two. Read
back through the zero-length wire the same triangle spans 64 ADC counts
against 63 codes, so the analog side follows. The NOISE generator's MAMP
is a MASK and behaves as one: 256 consecutive values span 15 codes at
MAMP 3 and 489 at MAMP 8, each inside its own `2^(MAMP+1) - 1`.

**The user offset calibration is a procedure and not a register.** With
the channel disabled and buffered, CEN set and OTRIM swept upward from
zero with 60 us (DS13560's tTRIM is 50) spent at each value, CAL_FLAG is
low at trim 0 and crosses at **14**, where this die's FACTORY trim is
**7** - the buffer comparing its own offset against VREF+/2, exactly as
16.4.12 describes. The factory value is restored bit for bit. Calibration
in an unbuffered mode is REFUSED, where 16.4.12 says it has no effect at
all.

**SAMPLE-AND-HOLD WITHOUT LSI DOES NOT FAIL - IT DEGRADES IN SILENCE.**
dac_hold_ck is LSI (table 85) and this driver deliberately does not
start it. Measured with LSIRDY read CLEAR: the pad carries the value to
within a count, exactly as the plain buffered mode does; the times land;
and **BWST never stands, at either state of the clock**. So there is no
bit an application can read to tell an armed sample-and-hold from a
degraded one - only the RCC knows. With LSI running the mode samples and
holds (the pad still carrying its value a whole hold time later with
nothing written in between), and that is indistinguishable from the
failure. What this desk cannot say is the thing the mode exists for:
16.4.6 sells it as a power saving, and that is a current measurement
with no meter here.

### The DAC as a comparator threshold (`test_stm32_analog` letter `m`)

`DacMode::internal_unbuffered` drives the on-chip peripherals and NO
pad - PA4 is not claimed at all - and `CompNegative::dac_channel1` is
what receives it. Measured against a free pad settled at about 1.22 V:
VALUE reads 1 with the DAC at code 0 and 0 at 4095, and a binary search
locates the crossing to one LSB. Channel 2 goes the same way as the
upper limit of a window comparator built from both channels at once, so
**both internal connections work on silicon** and neither costs a
pad. The comparator side of the story, including what this arrangement
can and cannot measure about the comparator itself, is in
[comp.md](comp.md).

- **Every row of 16.4.8's trigger multiplexer moves this converter**
  (letter `p`), one event at a time so the step is countable: TIM1,
  TIM2, TIM3, TIM7 and TIM15's TRGO each on one software update event
  (a fresh code in DHR, DOR proven still holding the old one, the
  trigger, DOR at the new one and the pad reading it back through
  ADC_IN4); both LPTIM outputs, which are waveforms rather than strobes
  and are started, allowed one edge and stopped; and **EXTI 9 through a
  pull-walked PC9**, the one row of the multiplexer that is a PAD, with
  the line's port selected in the EXTI and its sense rising - PC9 and
  not PB9, which the desk's I2C self-link holds up
  ([../bench.md](../bench.md)).
- **The DMA underrun, staged**: a converter asking for a DMA with NO
  channel armed at all raises DMAUDR on the trigger after the first,
  and the flag is clear before. It is write-one-to-clear and comes
  down. With DMAUDRIE set and the NVIC line open the same starvation
  reaches the vector the DAC shares with TIM6 and LPTIM1, once per
  unserved trigger (nine calls over a 900 us window at 10 kHz). **The
  flag has to be read with the interrupt OFF** - a handler that serves
  it clears it, so read with the interrupt armed the flag is zero and
  the evidence is the handler count instead (eighteen calls, the same
  fact seen from the wrong side).

## On the STM32G071RB

`test_stm32_analog` runs on the Nucleo-G071RB (DEV_ID 0x460, REV_ID
0x2000) and every DAC letter passes there unchanged: two channels, the
four data placements, both outputs, the wave generators, sample-and-hold,
the user trim and the DMA underrun.

**ES0418 2.7.1 is this part's own item and it does not bite**: "writing a
value different from 000 to the DAC channel MODE bitfield before
performing data initialization causes the corresponding DAC channel
analog output to be invalid", present on both revisions with a
workaround. `configure()` writes MCR.MODE with the channel disabled and
BEFORE any data write, which is the erratum's exact condition, and the
suite's letters - which read the DAC back through the ADC on PA4 -
measure a correct output every time. NOT REPRODUCED under
configure-then-set on DEV_ID 0x460 REV_ID 0x2000, which is not a
disproof.

The obligation is nonetheless STATED on `configure()`, because a driver
that cannot enforce a caller's ordering must say so: the workaround is
one write to any data register first, then the MODE, and `set(ch, code)`
is that write - it stores DHR12Rx unconditionally, with or without a
configuration. The data write stays outside `configure()`, so the
ordering is a stated obligation and not a hidden one.

ES0418 2.7.2 (the DMA underrun flag missed when an internal trigger lands
on the same cycle as a DMA request acknowledge) needs software AND
hardware triggers used concurrently; no letter of this suite does that,
so the condition is not reached. 2.3.1 (a GPIO assigned to a DAC channel
cannot be an output while that channel is in an on-chip-only mode) is not
reached either: the one letter that uses `DacMode::internal_unbuffered`
drives no pad while it holds.

## On the STM32G031K8

**THERE IS NO DAC ON THE G031** (DEV_ID 0x466): 16.3's table gives the
block to the G05x class and up, the header declares no `DAC1_BASE`, and
`dac_present()` is false - so `Dac` cannot be named at all and
`test_stm32_analog`'s DAC letters are compiled out there, each printing
that fact when its letter is invoked. Two consequences elsewhere on that
board: the suite's second analog source becomes the junction sensor
([adc.md](adc.md)), and TIM6's line - which carries the DAC's interrupt
where there is one - is LPTIM1's alone (the reserve's
`BRIO_STM32G0_TIM6_HANDLER` is not even defined on a part with no TIM6,
which is how a suite finds out at compile time rather than in a
Default_Handler spin).

## Not covered yet

**Driver gaps:**

- Nothing here turns on LSI, which is `dac_hold_ck` and therefore the
  clock the sample-and-hold mode runs on (table 85). That is deliberate
  - an RCC root is not this driver's to start - and the CONSEQUENCE is
  measured rather than guessed: with LSI stopped the mode degrades to
  plain buffered in complete silence (above). The header says so.

**Implemented but not bench-verified:**

- The wave generators from a HARDWARE trigger, and therefore at a rate
  worth a spectrum. Both are run one software step at a time, which is
  what makes them countable and is not the same measurement - and the
  trigger rows themselves are all measured (above), so what is left is
  only the RATE.
- What sample-and-hold COSTS. The mode runs and holds (above); 16.4.6
  sells it as a power saving and this bench has no current meter.
- The dual holding registers as a two-channel stream: `write_dual()` is
  exercised, `data_address_dual_12r()` has never carried a DMA channel.
