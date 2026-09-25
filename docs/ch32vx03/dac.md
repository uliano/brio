# DAC (CH32V303)

Two 12-bit digital-to-analog channels in one block, each with its own
enable, its own output buffer, its own trigger, its own wave generator
and its own DMA request - and no status register and no interrupt at
all. The CH32V303's alone: every part of that series has the block and
no CH32V203 has any. Documents of record: the CH32F/V20x_V30x_V31x
reference manual V2.3 (17.2.2.1 to 17.2.2.5 for the enable, the buffer,
the data formats, the DMA and the triggers with table 17-1, 17.2.3 for
when a datum reaches the output, 17.2.4 and 17.2.5 with figure 17-5 for
the two generators, 17.3 for the dual modes, 17.4 with table 17-2 for
the registers, 11.2.3's table 11-3 for the two DMA requests, 3.4.5 and
3.4.8 for the reset line and the gate) and the CH32V303 datasheet V3.5
(table 2-1-1 for which parts have the block and which timers can
trigger it, tables 3-1 and 3-4 for the pads, table 4-45 for what the
converter is electrically). Driver:
[brio/ch32vx03/dac.hpp](../../brio/ch32vx03/dac.hpp). Reference suite:
`test_vx03_dac`.

## What the silicon does

### Two channels, two pads, and the converter behind them

DAC1 drives PA4 and DAC2 drives PA5 on every package, and those two
pads are also the ADC's inputs 4 and 5 - so the converter reads the DAC
through the bond pad they share, which makes a DAC-into-ADC
measurement possible with no wire at all. There is no internal route: no
register of this chapter sends a channel anywhere but its pad. 17.2.2.1
asks for the pad in analog mode BEFORE the channel is enabled, to keep
the input stage from drawing current; that is `DacOut<ch>::claim()`.
PA4 and PA5 are also SPI1's pads, so a program with both blocks decides
which one owns them.

THE NUMBERING IS A TRAP IN ONE PLACE. The pinout figures of the
datasheet's 3.1 label PA4 "DAC0" and PA5 "DAC1"; the pin table 3-1, the
alternate-function table 3-4 and the whole of the manual's chapter call
them DAC1_OUT and DAC2_OUT, channel 1 and channel 2 - and so do this
driver's names, the DMA table's rows and the register fields.

### The data register is not the output register

A datum is written to a HOLDING register - nine spellings of it (17.2.2.3,
17.4.3 to 17.4.11): 12 bits right-aligned, 12 bits left-aligned from bit
4 and 8 bits that land in bits 11:4, per channel, and the same three for
both channels in one word - and reaches DAC_DORx, which is read-only, when
the channel's trigger says so: one PB1 cycle after the write with no
trigger selected, one PB1 cycle after the software trigger, and THREE
PB1 cycles after a hardware trigger (17.2.2.5, 17.2.3). The output is
then valid a settling time later - 3 to 4 us from one end of the scale
to the other (table 4-45).

Every placement writes the SAME internal register: a datum written
left-aligned or as a byte reads back through the right-aligned holding
register shifted into place, and so does a dual write through each
channel's own (measured). The dual registers take both channels in one
store, and one of them has a shape of its own: the left-aligned pair sits
in bits 15:4 and 31:20, not sixteen apart like every other pair in the
block.

### The buffer is disabled by a one

BOFFx is an output buffer DISABLE (17.4.1). With the buffer the channel
drives a load of 5 kOhm and more and stays 0 to 8 mV off ground and at
least 3.29 V of 3.3 at the top; without it the load must be 15 kOhm and
more and the rails come closer - 0 to 3 mV, 3.295 V (table 4-45).

### The triggers

TENx enables a trigger and TSELx picks one of table 17-1's eight codes:
the TRGO of TIM6 (000), TIM8 (001), TIM7 (010), TIM5 (011), TIM2 (100)
and TIM4 (101), EXTI line 9 (110) and the software bit SWTRIGx (111).
A rising edge is a trigger; SWTRIGx is cleared by the hardware once the
holding register has been taken, and it is write-only. TSELx cannot be
changed while ENx is set (17.2.2.5's note).

A CODE NAMES A TIMER THE PART MAY NOT HAVE. Every CH32V303 has TIM2 and
TIM4; the CH32V303RC and VC have TIM5 to TIM8 as well, and the 128 KB
CH32V303CB and RB have none of the four (table 2-1-1). A trigger whose
timer is absent is refused.

EXTI LINE 9 REACHES THE CONVERTER THROUGH ITS EVENT ENABLE, and not its
interrupt enable - the path the ADC's code 110 takes too ([adc.md](adc.md)),
stated in neither chapter and measured on both blocks.

### The two generators

Both step on a trigger and both need TENx (17.2.4's and 17.2.5's notes),
and both add their value to the holding register with the carry out of
bit 11 thrown away. Both are sized by MAMPx, which must be written
before the channel is enabled (17.2.4's note 2), and both are reset by
writing WAVEx back to 00.

- **The triangle** is a counter from 0 up to 2^(MAMP+1) - 1 and back
  down, one step a trigger; the counter is ADDED and then updated
  (17.3.3), so the first trigger outputs the holding register itself,
  and each end of the walk is output once before it turns.
- **The noise** is a twelve-bit linear feedback shift register preloaded
  with 0xAAA. Figure 17-5 draws it shifting RIGHT, with the exclusive-or
  of bits 6, 4, 1 and 0 - and the NOR of all twelve, the anti-lock that
  injects a one into a register at zero - fed into bit 11. MAMPx masks it
  to its low MAMP + 1 bits; the register is added and then updated
  (17.3.1), so the first trigger adds the masked preload. The figure is
  the silicon's register trigger for trigger (measured).

### The DMA request, and where it is served

DMAENx raises a request on every HARDWARE trigger - "software trigger
not included" (17.2.2.4) - after which the holding register can be
refilled. The requests are DMA2's: DAC1 on its channel 3 and DAC2 on its
channel 4 (table 11-3) - and channel 3 is the one WCH's own two DAC
examples use, for a single channel and for the dual register. There is no
underrun flag and no status register at all (table 17-2 has thirteen
words and not one of them is a flag): a stream that falls behind simply
converts the same datum twice.

The dual holding registers take both channels from ONE request: DAC1's
request feeding DAC_RD12BDHR a word a trigger, with both channels on the
same trigger and DAC2's request never raised - the arrangement of WCH's
own dual example, and one each channel follows on every trigger.

### Both channels at once

17.3's eleven dual modes are combinations of each channel's own fields:
independent or simultaneous triggers, the same or different generators.
The V2.3 revision of the manual closed 17.4.1 with a note that with BOTH
channels enabled "the same wave is output to 2 hardware channels
according to the configuration of channel1"; the V2.5 revision deletes
the note and 17.3 says the opposite throughout - and the silicon agrees
with the later text: with both channels enabled, a triangle on channel 1
and a fixed code on channel 2 each came out on its own pad. This driver
writes each channel's own fields.

### The reference, the gate and the vector that is not there

The output is VDDA x DOR / 4096 by 17.2.3 and VREF+ x DOR / 4096 by the
datasheet (table 4-45), which is the same number on every package but
the LQFP100, whose VREF+ pad the evaluation board ties to VDDA
([adc.md](adc.md)). The gate is DACEN in RCC_PB1PCENR and the block has
a reset line, DACRST in RCC_PB1PRSTR (3.4.8, 3.4.5). No vector: the
chapter has no interrupt source of its own.

## Types and verbs

### The vocabulary

`DacTrigger` (table 17-1's eight codes by the code that selects them)
with `dac_trigger_timer` (which timer a code names, 0 for the two that
name none) and `dac_trigger_valid` (whether this part has that timer),
`dac_exti_line`; `DacWave` (none, noise, triangle) and
`dac_wave_amplitude(mamp)` - the 2^(MAMP+1) - 1 both generators are
sized by, every code from 11 up meaning 4095 - with `dac_lfsr_preload`;
`DacFormat` (right12, left12, right8) with `dac_format_bytes` and
`dac_dual_format_bytes`, the element width a stream's engine must have;
`dac_dor_latency_pb1_cycles`; `DacChannelConfig` - `buffered` (true by
default, BOFFx inverted once here), `triggered` and `trigger`, `wave`
and `amplitude`, `dma` - with `dac_channel_config_valid()` refusing a
generator with no trigger, a DMA on the software trigger, a five-bit
amplitude and a timer the part has not got; `dac_dma_slot(ch)`,
`dac_pad(ch)` and `dac_adc_channel(ch)`.

### The pads

`DacOut<1|2>`: the pad (`pad`, `pin`), the converter input it also is
(`adc_channel`), and `claim()`, the analog mode 17.2.2.1 asks for.
Refused on a part without the DAC.

### The converter

`Dac`, a monostate - one block on every part that has one - spelled
`DacUnit<has>` inside a template that must compile on both series. The
block: `init()` (the gate and the reset line, both channels off) and
`release()`, `bus_clock()`, `reset()`. A channel, 1 or 2: `configure(ch,
config)` and the compile-time `configure<ch, config>()` whose refusals
are static_asserts, `configuration(ch)` reading it back, `enable(ch,
on)` / `enabled(ch)`, `wave(ch, w)` for the generator alone and
`dma(ch, on)` for the request alone. CONFIGURE LEAVES THE CHANNEL OFF:
TSELx and MAMPx take a write only with ENx clear, so the verb drops the
enable first and the caller enables after, which is also where the
channel's wake-up time goes (6.5 to 10 us, table 4-45). The data:
`write(ch, code)`, `write_left(ch, value)`, `write8(ch, code)`,
`write_dual(code1, code2)`, `write_dual_left()`, `write_dual8()`;
`code(ch)` reads the 12-bit holding register back - every placement
writes the same internal register - and `output(ch)` reads DOR, which
with a generator running is the holding register plus the generator's
value. `software_trigger(ch)` and `software_trigger_both()`. The
stream: `data_address(ch, format)` and `dual_data_address(format)`, the
registers a DMA writes (the format decides the beat), `dma_slot(ch)`,
and `claim_stream<ch, Engine, format>()` / `claim_dual_stream<Engine,
format>()`, which arm an engine on that register and set DMAENx in one
verb, refused at compile time for an engine on any slot but the
request's own and for an element whose width is not the format's. The
arithmetic: `code_for_mv(mv, ref)` and `mv_for_code(code, ref)`,
[util/analog.hpp](../../brio/util/analog.hpp)'s `dac_code` and `dac_mv`
over this converter's full scale.

## How to use it

A level on PA4, read back by the converter on the same pad:

```cpp
brio::Dac::init();
brio::DacOut<1>::claim();                         // PA4, analog, before the enable
(void)brio::Dac::configure(1, brio::DacChannelConfig{});   // buffered, no trigger
(void)brio::Dac::enable(1, true);
(void)brio::Dac::write(1, brio::Dac::code_for_mv(1650, vdda_mv));

brio::Adc<1>::select_channel(brio::DacOut<1>::adc_channel);   // channel 4
const uint16_t counts = brio::Adc<1>::read_settled(4);
```

A datum held until the software says so:

```cpp
(void)brio::Dac::configure(1, {.triggered = true, .trigger = brio::DacTrigger::software});
(void)brio::Dac::enable(1, true);
(void)brio::Dac::write(1, 2048);       // held
(void)brio::Dac::software_trigger(1);  // DOR takes it one PB1 cycle later
```

A table played for ever, one entry a timer update, by DMA2's channel 3 -
TIM6's TRGO where the part has the basic timer, TIM4's on the 128 KB
parts:

```cpp
using Row = brio::DmaRequestOf<brio::DmaRequest::dac1>;
using Player = brio::DmaLoopEngine<Row::controller, Row::channel, uint16_t>;

brio::Dac::configure<1, brio::DacChannelConfig{.triggered = true,
                                               .trigger = brio::DacTrigger::tim6_trgo,
                                               .dma = true}>();
brio::Dac::claim_stream<1, Player>();
(void)Player::start(table, 8);
(void)brio::Dac::enable(1, true);
(void)brio::Tim<6>::master(brio::TimMasterMode::update);   // TIM6's TRGO is code 000
brio::Tim<6>::enable(true);

extern "C" BRIO_CH32_INTERRUPT void dma2_channel3_handler() {
    const uint8_t f = Player::service();
    if ((f & Player::flag_complete) != 0u) { Player::lap(); }
    if ((f & Player::flag_error) != 0u) { Player::fail(); }
}
```

Both channels from one stream, a word a trigger:

```cpp
using DualPlayer = brio::DmaLoopEngine<Row::controller, Row::channel, uint32_t>;

(void)brio::Dac::configure(1, {.triggered = true, .trigger = brio::DacTrigger::tim6_trgo});
(void)brio::Dac::configure(2, {.triggered = true, .trigger = brio::DacTrigger::tim6_trgo});
brio::Dac::claim_dual_stream<DualPlayer>();       // DAC1's request, DAC_RD12BDHR
(void)DualPlayer::start(pairs, n);                // (code2 << 16) | code1
```

A triangle of amplitude 1023 over 512, stepped by a timer:

```cpp
(void)brio::Dac::configure(1, {.triggered = true, .trigger = brio::DacTrigger::tim2_trgo,
                               .wave = brio::DacWave::triangle, .amplitude = 9});
(void)brio::Dac::write(1, 512);
(void)brio::Dac::enable(1, true);
```

## Bench findings

`test_vx03_dac` on the CH32V303VCT6, with the PLL on the HSI at 96 MHz
so that the converter judging the pads is in specification (ADCCLK
12 MHz) and nothing wired: **26 pass, 0 fail**. The judge is
ratiometric - the DAC's output and the ADC's count are both against the
same supply, measured at 3299 to 3303 mV through VREFINT - so a code and
the count it reads back are compared directly.

- **The six placements land in one register per channel.** 0xABCD and
  0x789F written left-aligned read back as 0xABC and 0x789, 0x5A and
  0xC3 written as bytes as 0x5A0 and 0xC30, the dual left pair 0x3330 /
  0x4440 as 0x333 / 0x444 and the dual bytes 0x55 / 0x66 as 0x550 /
  0x660 - and each reached DOR, with no trigger selected, before the
  core's next read could look. A configuration of every field read back
  whole (DAC_CTLR 0x17AE for an unbuffered channel 1 on TIM4's trigger
  with a triangle of MAMP 7 and its DMA request), channel 2's half of
  the register untouched; a generator with no trigger, a DMA on the
  software trigger and a five-bit amplitude were refused with nothing
  written.
- **The pads, read by the converter.** Seven codes from 0 to 4095 on
  each pad: buffered, every mid-scale code read within 13 counts of
  itself (the worst PA5's 3597 for 3584), code 0 read 0 counts and 4095
  read 4084 and 4088 - some 10 mV under the supply; unbuffered, within
  10 counts, code 0 at 0 and 4095 at 4085 and 4086. Both well inside
  table 4-45's offset of 12 mV, gain error of 0.4 % and rails, and the
  buffered channel 2 running a few counts high through the middle of the
  scale (1028, 2053, 3074 for 1024, 2048, 3072) where the unbuffered one
  does not.
- **Every trigger holds the datum until it comes.** The software trigger:
  DOR still at the old value five microseconds after the write, and the
  new one right after SWTRIG1. The TRGO of all six timers table 17-1
  names - TIM2, TIM4, TIM5, TIM6, TIM7 and TIM8, codes 100, 101, 011,
  000, 010 and 001 - each held a datum written mid-period until its next
  update, 499 us later for a 1 ms period.
- **EXTI line 9 from a pad of our own**: an edge on the line merely
  sensed moved nothing, the same edge with the line's EVENT enable moved
  the datum, and with its INTERRUPT enable instead it did not - while the
  line's own handler ran for it.
- **The noise is figure 17-5's register.** The first trigger added the
  masked preload - 0x00A with MAMP 3, 0x8AA over a holding register of
  0x800 with MAMP 7, 0xAAA with MAMP 11 - and the twelve-bit run went on
  0xAAA, 0xD55, 0xEAA, 0xF55, 0xFAA, 0xFD5, 0xFEA, 0x7F5; sixty-four
  triggers at each of the three widths all equal to the model's register.
- **The triangle turns at both ends.** MAMP 2 over 0x400 gave 0, 1, 2,
  3, 4, 5, 6, 7, 6, 5, 4, 3, 2, 1, 0, 1 above the holding register: the
  counter added and then updated, so the first trigger outputs the
  holding register itself, and every step one count.
- **The stream: DMA2's channel 3 playing a table into DAC1.** Eight
  levels, one a TIM6 update at 1 kHz, for 20 ms: the channel's own
  handler counted two laps and no fault, and 3738 conversions of PA4 in
  those 20 ms found all eight levels, nine in ten of them on a level of
  the table - the rest caught mid-step.
- **The dual stream: one request, both channels.** DAC1's request alone
  (DMAEN1 set, DMAEN2 clear) feeding the dual holding register a word a
  TIM6 update: 1053 pairs of DOR1 and DOR2 read back to back were all
  1053 a pair of the table, and 1049 of 1053 pairs of conversions of the
  two pads summed to full scale as every pair of the table does.
- **Each channel follows its own configuration.** Both enabled, a
  triangle on channel 1 stepped 700 times and a fixed 3072 on channel 2:
  PA4 read 1207 against DOR1's 1211 and PA5 3067 against 3072 - and
  each channel alone drove its own pad the same way. V2.3's two-channel
  note does not hold on this silicon.

## Not covered yet

Implemented but not bench-verified, each with what would measure it:

- **Channel 2's generators, its software trigger and the simultaneous
  one** (`software_trigger(2)`, `software_trigger_both()`): the same bits
  sixteen apart as channel 1's, which letters d and e measure; the same
  two letters run on channel 2 would measure them.
- **DAC2's own request on DMA2's channel 4.** The dual stream rides
  DAC1's request by design; `claim_stream<2, Engine>()` is compiled and
  refused at compile time off its slot, and the stream letter run on
  channel 2 would measure it.
- **The three PB1 cycles a hardware trigger takes to reach DOR** (17.2.3,
  against one for the software trigger): below what a core's read of DOR
  through the bus can resolve, so the suite measures that the datum is
  HELD until the trigger and taken by the time the core looks, not the
  latency between; a logic analyser on the timer's TRGO and the pad
  would.
- **The 128 KB CH32V303CB and RB**, whose DAC takes only TIM2's and
  TIM4's TRGO, EXTI line 9 and the software trigger: compiled for both
  and asserted at compile time; what would measure them is a board.
