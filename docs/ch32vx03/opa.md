# OPA (CH32V203, CH32V303)

Up to four operational amplifiers - two on the CH32V203, four on the
CH32V303 - each with a positive input that is one of two pads, a
negative input that is one of two pads, and an output that is one of two
pads. That is the whole chapter: four bits an amplifier in ONE register,
no key, no lock, no interrupt, no event, no DMA, and no verb that is
refused while the block runs - plus, on the CH32V303, one high-speed bit
an amplifier in another register. Documents of record: the
CH32F/V20x_V30x_V31x reference manual V2.3 (30.2 for what the four bits
do, 30.3.1 for the register and for which amplifiers belong to which
device class, 33.2.2 for the high-speed bits), the CH32V203 datasheet
V2.8 (table 2-1 for how many a part has, 3.2's pin tables for which pad
each selection is, table 4-31 for what the amplifier is electrically)
and the CH32V303 datasheet V3.5 (table 2-1-1, the pin tables 3-1 and
3-4, and tables 4-46-1 and 4-46-2 for the amplifier in its two speeds).
Three device classes read the chapter: the CH32V20x_D6 for every
CH32V203 up to the CH32V203C8, the CH32V20x_D8 for the CH32V203RB, and
the CH32V30x_D8 for the four CH32V303 parts; the register's upper half
is OPA3 and OPA4, and every one of those bit descriptions names the last
class and the ones beside it. Driver:
[brio/ch32vx03/opa.hpp](../../brio/ch32vx03/opa.hpp). Reference suite:
`test_vx03_opa`.

## What the silicon does

### What it is not, which matters more than what it is

There is NO internal feedback network here and NO gain. The CH32V00x's
amplifier switches a 192 kOhm resistor in and offers gains of 4 to 32;
this one offers neither, so as WCH ships it on this family the amplifier
is an open-loop stage. The datasheets call it "operational
amplifier/comparator" and rate it at 136 dB of open-loop gain, which is
the same statement said twice: with a wire from an output pad back to a
negative input pad it is a follower or a divider, and with no wire at
all it is a COMPARATOR whose output saturates to a rail.

The gain a program can reach with no wire is the CONVERTER's (CTLR1's
PGA behind BUFEN, [adc.md](adc.md)), which sits in front of the
converter and not here.

### Where the output goes

There is no internal route: 30.2's "the output pin can select
general-purpose I/O or the ADC sampling channel" means the PAD, and the
converter reads the amplifier by converting that pad's own channel. The
first output of each amplifier is a converter input on every part -
PA3, PA2, PA1 and PA0, channels 3, 2, 1 and 0, for OPA1 to OPA4. The
SECOND output is where the two series part: on the CH32V203 it is PB1
(channel 9) for OPA1 and PA4 (channel 4) for OPA2, so every output pad
is a converter input; on the CH32V303 it is PE15, PE14, PE7 and PE8 for
OPA1 to OPA4, port-E pads that are NO converter's input and whose swing
the datasheet limits to 0 to 2 V at a VDDA of 3.3 V (table 4-46, note
3). `OpaOut<n, which>::adc_channel` is the channel, or 0xFF there, and
`reaches_adc` says which.

### The pads

The map is the device class's; what a PACKAGE decides is which of those
pads it brings out, which is `Pin`'s own refusal.

| | positive 0 | positive 1 | negative 0 | negative 1 | output 0 | output 1 |
|---|---|---|---|---|---|---|
| OPA1 | PB15 | PB0 | PB11 | PA6 | PA3 | PB1 (CH32V203), PE15 (CH32V303) |
| OPA2 | PB14 | PA7 | PB10 | PA5 | PA2 | PA4 (CH32V203), PE14 (CH32V303) |
| OPA3 | PB13 | PC5 | PB2 | PC2 | PA1 | PE7 |
| OPA4 | PB12 | PC4 | PB1 | PC3 | PA0 | PE8 |

OPA3 and OPA4 are the CH32V303's alone. Their CH1 inputs are port C's,
which the CH32V303CB's LQFP48 does not bring out, and every second
output of that series is port E's, which only the CH32V303VC's LQFP100
does.

### How many a part has

The CH32V203's table 2-1 gives every part two except the twenty-pin
one, which it gives one - and WHICH one is in the pin table rather than
the count: that package bonds neither PB15 nor PB0, which are OPA1's two
positive inputs, so the amplifier it has is OPA2. The CH32V303's table
2-1-1 gives every part four. `device::has_opa(n)` is that fact, and
`Opa<n>` does not compile for an amplifier the part has not got.

### The register

OPA_CTLR, one word, four bits an amplifier at 4 x (n - 1): the enable,
the output selection, the negative selection and the positive one. It
sits INSIDE THE EXTEN BLOCK'S WINDOW, between EXTEN_CTR and EXTEN_CTR2 -
which is the reserved word `device.hpp`'s `ExtenRegs` shows there - and
like EXTEN it has no clock gate in RCC and no reset line: a program
writes it and it is written.

### The high-speed bits

EXTEN_CTR2's four low bits are OPA1_HSMD to OPA4_HSMD (33.2.2), and the
CH32V303's datasheet rates the two speeds in a pair of tables: a
unity-gain bandwidth of 19 MHz against 53, a slew rate of 8 V/us against
16, 195 uA of supply against 770 (tables 4-46-1 and 4-46-2). The
register's note gives it to the CH32V30x_D8 and the classes beside it,
and only on lots whose sixth digit from the end is not zero - which a
program cannot read. No CH32V203 has it.

AND WHERE A DIE HAS NOT GOT THE REGISTER, ITS ADDRESS IS NOT EMPTY: IT
MIRRORS EXTEN_CTR. Measured on the CH32V303VCT6: 0x4002 3808 reads
EXTEN_CTR's own word (0xA50 - the reset value's regulator trims and
lock-up enable, and the HSI divider bit the clock task sets), and a
write there lands in EXTEN_CTR: a high-speed bit set there would set
EXTEN_CTR's USB bits, and a whole word would rewrite the regulator's
trims. So reading a bit back is NOT how a die says it has the register;
reading the WORD is - the real register keeps bits 31:4 clear and reads
unlike EXTEN_CTR, the mirror reads EXTEN_CTR - and nothing in the driver
writes the address before `opa_high_speed_present()` has told the two
apart.

## Types and verbs

`OpaPin` names the six pads an amplifier can reach in ONE enum
(`positive0`, `positive1`, `negative0`, `negative1`, `out0`, `out1`), so
that a configuration cannot be spelled with a pad in the wrong role and
only then refused; `opa_is_positive` / `opa_is_negative` /
`opa_is_output` and `opa_code` are the predicates over it, `opa_pad(n,
which)` the map above - reading the device class for the second outputs
and for the upper two amplifiers - `opa_output_channel(n, which)` the
ADC channel an output lands on, 0xFF for a pad that is no converter
input, and `opa_class_has_four` whether the class has OPA3 and OPA4.

`OpaConfig` holds the three selections with `opa_config_valid()` beside
it and `opa_config_bits()` for the four bits it makes.

`OpaIn<n, which>` and `OpaOut<n, which>` are the pads as types - the pad
itself, the `Pin` behind it, `claim()` for the analog mode, and on the
output the ADC channel and `reaches_adc`. Both refuse a code in the
wrong role, an amplifier this part has not got, and (through `Pin`) a
pad this package does not bond.

`Opa<1..4>` is the amplifier: `configure()` and `configuration()`,
`enable()` / `enabled()`, the three selections one at a time
(`positive`, `negative`, `output`) for a program that switches an input
while it runs, `field()` for the four bits as they read, `pad()` and
`adc_channel()` for what is selected now, `high_speed(on)` and
`high_speed()` over the amplifier's HSMD bit where the class has it
(`has_high_speed`; a compile error elsewhere) - `high_speed(on)`
answering false, with nothing written, where the DIE has not got the
register (`opa_high_speed_present()`, the section above) - and
`release()`, which puts this amplifier's four bits back - and its
high-speed bit, where there is one - and leaves the others alone.

## How to use it

An amplifier comparing two pads, read by the converter:

```cpp
using Pos = brio::OpaIn<2, brio::OpaPin::positive1>;    // PA7
using Neg = brio::OpaIn<2, brio::OpaPin::negative1>;    // PA5
using Out = brio::OpaOut<2, brio::OpaPin::out0>;        // PA2, channel 2

Pos::pin::output(true);
Neg::pin::output(false);
Out::claim();
(void)brio::Opa<2>::configure({.positive = brio::OpaPin::positive1,
                               .negative = brio::OpaPin::negative1,
                               .output = brio::OpaPin::out0});
brio::Opa<2>::enable(true);

brio::Adc<1>::select_channel(Out::adc_channel);
const uint16_t counts = brio::Adc<1>::read_settled(3);
```

One input moved while it runs, which is one bit:

```cpp
(void)brio::Opa<2>::positive(brio::OpaPin::positive0);   // now PB14
```

A CH32V303 amplifier on its second output, which no converter reads -
the pad read as a level instead, against the pull that would say the
opposite:

```cpp
using Out1 = brio::OpaOut<3, brio::OpaPin::out1>;       // PE7
static_assert(!Out1::reaches_adc);
(void)brio::Opa<3>::output(brio::OpaPin::out1);
Out1::pin::input(brio::PinPull::down);
const bool high = Out1::pin::read();
```

And its high-speed mode, which is a question to the die:

```cpp
if (!brio::Opa<3>::high_speed(true)) {
    // no EXTEN_CTR2 on this die: nothing was written, and the address mirrors EXTEN_CTR
}
```

An amplifier whose inputs come from outside and whose output goes to a
pad rather than to the converter: the same three selections, and the
output pad left alone - nothing here claims a pad the program has not
handed over.

## Bench findings

`test_vx03_opa` measures on both parts with the PLL on the HSI at 96 MHz
(the converter that reads the amplifier is in specification there,
[adc.md](adc.md)). NOTHING OUTSIDE THE CHIP: both inputs are pads driven
by their own port and the output is read as an analog channel - or, on
the CH32V303's port E, as a level. On the CH32V203C8T6 the five letters
of that part: **21 verdicts in `z`**. On the CH32V303VCT6 all eight:
**30 pass, 0 fail**, with the evaluation board's own wires on several of
those pads (each to another pad left a floating input, which the driven
pad drives with it). Where one number is given below it is the
CH32V203C8T6's unless the CH32V303VCT6 is named.

- **The amplifier is a comparator with no wire, and it saturates.** With
  its positive input at the rail and its negative at ground the output
  read 4095 counts = 3300 mV; with the two swapped, 9 counts = 7 mV; and
  it followed them back. The high saturation is 1 mV under a supply
  measured at 3303 mV, where the datasheet rates VOHSAT at VDDA-45 mV
  into 4 kOhm and VDDA-10 mV into 20 kOhm - there is no load here but
  the converter's own sampling. On the CH32V303VCT6: 4090 to 4091 counts,
  4 mV under a supply of 3299 mV, and 0 counts the other way.
- **The output reaches the converter THROUGH THE PAD.** Both of OPA2's
  output pads and both of OPA1's were read on their own ADC channels;
  moving the selection from one output to the other moved the level with
  it, and the pad it left stopped being driven (4095 -> 153 counts while
  the newly selected one took 9).
- **Each selection is a pad and the output follows it.** OPA2 with its
  CH0 pair driven high-over-low read 4095 counts, and the same amplifier
  switched to its CH1 pair, driven low-over-high, read 9 - one bit each,
  with nothing else touched.
- **The first amplifier behaves the same on its own six pads**: 4095 and
  9 counts on channel 3 for its CH1 pair, 4095 on channel 9 for its CH0
  pair through its other output - and on the CH32V303VCT6 4091 and 0 on
  channel 3, its CH0 pair HIGH on PE15.
- **Four bits an amplifier, and they do not collide.** Both fields read
  zero out of reset; all eight selections of one amplifier were written
  and read back; configuring and enabling one left the other's four bits
  exactly as they were, and releasing one left the other running.
- **The amplifier drives its pad against that pad's own pull.** With the
  output pad left a pulled-DOWN input - a 40 kOhm source of its own - it
  read 9 counts with the amplifier off and 4043 with it on (0 and 4039 on
  the CH32V303VCT6). The
  datasheet's 600 uA of drive against 40 kOhm is not a contest, and it
  says in passing that the output reaches the pad whatever the pad's
  digital mode is.
- **A disabled amplifier drives nothing**, and its output pad then
  floats: it read 943 counts before anything had driven it and held 4095
  after being driven there, which is a floating pad's charge and not a
  level (732 to 901 and 4093 on the CH32V303VCT6).
- **OPA3 and OPA4 on the CH32V303VCT6**: each saturated to both rails on
  its first output from its CH0 pair (4091 or 4092 and 0, on channels 1
  and 0), its CH1 pair moved the same output the other way, and its
  second output carried both levels on port E where no converter channel
  reads it.
- **THE PORT-E OUTPUTS SWING FAR ENOUGH TO BE READ AS LEVELS**
  (CH32V303VCT6): PE14, PE15, PE7 and PE8, each taken against the pull
  that says the opposite, read HIGH when their amplifier drove high and
  LOW when it drove low - so their swing, rated at 0 to 2 V at a VDDA of
  3.3 V (table 4-46, note 3), clears the 1.63 V VIH of those five-volt
  tolerant pads; and the first output each amplifier left behind was
  undriven at once.
- **THIS DIE HAS NO EXTEN_CTR2, AND ITS ADDRESS IS EXTEN_CTR'S**
  (CH32V303VCT6): both read 0xA50; each of the four high-speed verbs
  answered false and EXTEN_CTR read the same after all four; OPA_CTLR was
  untouched by any of it.
- **An output follows a rail-to-rail step of its inputs' difference in a
  microsecond or so** (CH32V303VCT6): the two inputs of each CH0 pair
  swapped in one store of port B's output register, and the output pad's
  own digital input polled: 65 to 77 core cycles at 96 MHz to see a rise
  and 122 to 131 to see a fall, on all four amplifiers - some 0.7 and 1.3
  us with the pad's synchronizer and the poll included, against the
  datasheet's slew of 8 V/us.

## Not covered yet

Driver gaps, each with its reason:

- **The amplifier as an AMPLIFIER.** A follower, a divider or any closed
  loop needs a wire from an output pad to a negative input pad, because
  this block has no internal feedback; with none strapped the only
  honest measurement is the open-loop one above. One jumper - PA2 to
  PA5, or PA3 to PA6 - is what would turn every letter of the suite into
  a gain measurement.
- **The input offset voltage** (1.5 mV typical and 6 mV maximum on the
  CH32V203, 2.5 and 10 mV on the CH32V303): a property of the closed
  loop, so the same wire, plus a source between the rails to measure it
  against.
- **The common-mode input range and the drive into a real load** (the
  CH32V203's table 4-31, the CH32V303's 4-46: CMIR, ILOAD, RLOAD): both
  want something attached to the output pad.
- **The amplifier as a TIMER's input.** The datasheets say the
  comparison result "can be output by GPIO or directly connected to the
  input channel of TIMx"; no register of this chapter selects such a
  route, and the timer chapter's own input tables do not name the
  amplifier. What the sentence means on this family is not established
  here; what a suite could show is the PAD PATH - the output pad IS a
  timer channel pad on some of these pins, and a pad driven by one
  peripheral reaches another's input on this family
  ([tim.md](tim.md)) - which would prove a route through the pin and
  not the internal one the datasheet's sentence claims.

Implemented but not bench-verified, each with what would measure it:

- **The high-speed mode on a die that has EXTEN_CTR2.** The verbs ask the
  die, and the CH32V303VCT6 has not got the register; each HSMD bit set
  and read back alone, and each amplifier's step response in both modes,
  are what `test_vx03_opa`'s letter h measures on a die of a lot whose
  penultimate sixth digit is not zero.
- **The parts other than the CH32V203C8 and the CH32V303VC.** Which
  amplifiers a part has and which of its pads it bonds fold through its
  own table, and the whole stratum compiles for all thirteen both ways
  the hardware prologue can be built (`brio check ch32vx03`); the
  twenty-pin part's single amplifier and the CH32V303CB's missing CH1
  inputs of OPA3 and OPA4 are asserted at compile time and measured on
  no board. What would measure them is a board.
