# DAC (STM32G0)

> **PROVISIONAL.** The whole of chapter 16's register surface is
> implemented - both channels, the eight output modes, the trigger
> table, the three data formats and the dual holding register, the DMA
> requests with their underrun report, the offset calibration and the
> sample-and-hold times - and the parts a bench with no wires can reach
> are verified: the transfer curve, the buffer's swing, the formats, and
> a DMA-fed waveform paced by a timer. The wave generators, the
> sample-and-hold mode and the user offset trim are written and NOT
> measured. The list is in "Not covered yet".

Documents of record: RM0444 Rev 6 ch. 16, with the DMAMUX request table
55 and the vector table 12.3 (table 61); DS13560 Rev 5 table 12 (the
pads) and table 64 (the DAC's electrical characteristics); errata
ES0548 Rev 3, which has **no item touching the DAC** on either silicon
revision. Driver: `stm32g0/dac.hpp`; the reference vocabulary is
`stm32g0/vref.hpp`'s. Bench suite: `test_stm32_analog` (shared with the
ADC, the reference buffer and the comparators). Family fixture
`test/family_stm32g0/dac.cpp` plus one negative under
`tools/check_stm32g0.sh`.

## What the silicon does

**Two 12-bit channels, and on the G031/G041 there are none at all.**
16.3's own table says so and the device header agrees by declaring no
`DAC1_BASE` there, so `Dac` is a compile-time refusal on those parts and
a MONOSTATE on the others (there is one DAC wherever there is one).
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
without one rather than quietly doing nothing - the samc `dac.hpp`
ruling on dithering, reached again from another chapter. MAMPx is
2^(code+1) - 1 in both its readings (an LFSR mask and a triangle
amplitude), saturating at 4095 above code 11.

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

**LD4 is not a load the DAC can feel, and that was a surprise.** Both
channels run at once, one on the free PA4 and one on PA5 = LD4, and the
two agree to **5 mV at every code** across the range - the worst
difference is at code 1024, not at the top where a conducting LED would
droop the buffer. PA5 also follows a 40 kohm internal pull between the
rails exactly as the free pads do. So whatever drives LD4 on this board
is high-impedance seen from the pin, and the obvious guess - a diode and
a series resistor to ground - is wrong. A DAC-driven brightness works
either way, and it is the one thing in this suite a human can see.

**One trigger, both converters, no CPU:** the chain is described in
[adc.md](adc.md)'s bench findings - TIM6's TRGO starting the DAC and the
ADC on the same edge, a `DmaLoopEngine` playing a 16-entry table into
DHR12R1 and a `DmaPingPongEngine` draining ADC_DR, with zero samples off
the table over six blocks and every seam stepping by exactly 8. The one
DAC-side lesson it paid for: **the launch block is out of phase by
construction**, because 16.4.8's rule makes the first datum the CPU's
and not the table's.

## Not covered yet

**Driver gaps:**

- Nothing here turns on LSI, which is `dac_hold_ck` and therefore the
  clock the sample-and-hold mode runs on (table 85). The times are
  writable and the mode is selectable; a caller that asks for it with
  LSI stopped gets a channel that never samples, and the header says so
  rather than reaching into the RCC.

**Implemented but not bench-verified:**

- The NOISE and TRIANGLE wave generators. Both are configured and
  refused correctly without a trigger; neither has been run, and
  measuring one properly wants a spectrum or a long capture, not a
  handful of conversions.
- Sample-and-hold mode, for the same reason as the LSI gap above.
- The USER offset calibration (16.4.12). `calibration_mode()`,
  `calibration_flag()` and the trim are exercised as registers; the
  successive-approximation sweep itself needs a tTRIM delay from
  DS13560 and a reason, and the factory trim is loaded at reset.
- The DMA UNDERRUN path. `underrun()` is read on every chain run and has
  never been true, which is the right answer for a stream that keeps up
  and no evidence at all about the flag.
- Eight of the nine hardware triggers, and EXTI9 among them.
- The dual holding registers as a two-channel stream: `write_dual()` is
  exercised, `data_address_dual_12r()` has never carried a DMA channel.
