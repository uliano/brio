# OPA (CH32V203)

Two operational amplifiers, each with a positive input that is one of
two pads, a negative input that is one of two pads, and an output that
is one of two pads. That is the whole chapter: four bits an amplifier in
ONE register, no key, no lock, no interrupt, no event, no DMA, and no
verb that is refused while the block runs. Documents of record: the
CH32F/V20x_V30x_V31x reference manual V2.3 (30.2 for what the four bits
do, 30.3.1 for the register and for which amplifiers belong to which
device class) and the CH32V203 datasheet V2.8 (table 2-1 for how many a
part has, 3.2's pin tables for which pad each selection is, table 4-31
for what the amplifier is electrically). Ours is the CH32V20x_D6 device
class for every part up to the CH32V203C8 and CH32V20x_D8 for the
CH32V203RB; the register's upper half is OPA3 and OPA4 and every one of
those bit descriptions names other families. Driver:
[brio/ch32v203/opa.hpp](../../brio/ch32v203/opa.hpp). Reference suite:
`test_v203_opa`.

## What the silicon does

### What it is not, which matters more than what it is

There is NO internal feedback network here and NO gain. The CH32V00x's
amplifier switches a 192 kOhm resistor in and offers gains of 4 to 32;
this one offers neither, so as WCH ships it on this family the amplifier
is an open-loop stage. The datasheet calls it "operational
amplifier/comparator" and rates it at 136 dB of open-loop gain, which is
the same statement said twice: with a wire from an output pad back to a
negative input pad it is a follower or a divider, and with no wire at
all it is a COMPARATOR whose output saturates to a rail.

The gain a program can reach with no wire is the CONVERTER's (CTLR1's
PGA behind BUFEN, [adc.md](adc.md)), which sits in front of the
converter and not here.

### Where the output goes

Every OPA output pad is also an ADC input pad. OPA1 reaches PA3
(channel 3) or PB1 (channel 9); OPA2 reaches PA2 (channel 2) or PA4
(channel 4). That is what 30.2 means by "the output pin can select
general-purpose I/O or the ADC sampling channel": there is no internal
route, the pad IS the route, and the converter reads the amplifier by
converting that pad's own channel. `OpaOut<n, which>::adc_channel` is
that number.

### The pads

The map is the family's and the same on every part; what a PACKAGE
decides is which of those pads it brings out, which is `Pin`'s own
refusal.

| | positive 0 | positive 1 | negative 0 | negative 1 | output 0 | output 1 |
|---|---|---|---|---|---|---|
| OPA1 | PB15 | PB0 | PB11 | PA6 | PA3 | PB1 |
| OPA2 | PB14 | PA7 | PB10 | PA5 | PA2 | PA4 |

### How many a part has

Datasheet table 2-1 gives every part two except the twenty-pin one,
which it gives one - and WHICH one is in the pin table rather than the
count: that package bonds neither PB15 nor PB0, which are OPA1's two
positive inputs, so the amplifier it has is OPA2. `device::has_opa(n)`
is that fact and `Opa<1>` does not compile there.

### The register

OPA_CTLR, one word, four bits an amplifier at 4 x (n - 1): the enable,
the output selection, the negative selection and the positive one. It
sits INSIDE THE EXTEN BLOCK'S WINDOW, between EXTEN_CTR and EXTEN_CTR2 -
which is the reserved word `device.hpp`'s `ExtenRegs` shows there - and
like EXTEN it has no clock gate in RCC and no reset line: a program
writes it and it is written.

## Types and verbs

`OpaPin` names the six pads an amplifier can reach in ONE enum
(`positive0`, `positive1`, `negative0`, `negative1`, `out0`, `out1`), so
that a configuration cannot be spelled with a pad in the wrong role and
only then refused; `opa_is_positive` / `opa_is_negative` /
`opa_is_output` and `opa_code` are the predicates over it, `opa_pad(n,
which)` the map above, and `opa_output_channel(n, which)` the ADC
channel an output lands on.

`OpaConfig` holds the three selections with `opa_config_valid()` beside
it and `opa_config_bits()` for the four bits it makes.

`OpaIn<n, which>` and `OpaOut<n, which>` are the pads as types - the pad
itself, the `Pin` behind it, `claim()` for the analog mode, and on the
output the ADC channel. Both refuse a code in the wrong role, an
amplifier this part has not got, and (through `Pin`) a pad this package
does not bond.

`Opa<1|2>` is the amplifier: `configure()` and `configuration()`,
`enable()` / `enabled()`, the three selections one at a time
(`positive`, `negative`, `output`) for a program that switches an input
while it runs, `field()` for the four bits as they read, `pad()` and
`adc_channel()` for what is selected now, and `release()`, which puts
this amplifier's four bits back and leaves the other's alone.

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

An amplifier whose inputs come from outside and whose output goes to a
pad rather than to the converter: the same three selections, and the
output pad left alone - nothing here claims a pad the program has not
handed over.

## Bench findings

`test_v203_opa`, 21 verdicts in `z`, on the CH32V203C8T6 with the PLL on
the HSI at 96 MHz (the converter that reads the amplifier is in
specification there, [adc.md](adc.md)). NOTHING OUTSIDE THE CHIP: both
inputs are pads driven by their own port and the output is read as an
analog channel.

- **The amplifier is a comparator with no wire, and it saturates.** With
  its positive input at the rail and its negative at ground the output
  read 4095 counts = 3300 mV; with the two swapped, 9 counts = 7 mV; and
  it followed them back. The high saturation is 1 mV under a supply
  measured at 3303 mV, where the datasheet rates VOHSAT at VDDA-45 mV
  into 4 kOhm and VDDA-10 mV into 20 kOhm - there is no load here but
  the converter's own sampling.
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
  pair through its other output.
- **Four bits an amplifier, and they do not collide.** Both fields read
  zero out of reset; all eight selections of one amplifier were written
  and read back; configuring and enabling one left the other's four bits
  exactly as they were, and releasing one left the other running.
- **The amplifier drives its pad against that pad's own pull.** With the
  output pad left a pulled-DOWN input - a 40 kOhm source of its own - it
  read 9 counts with the amplifier off and 4043 with it on. The
  datasheet's 600 uA of drive against 40 kOhm is not a contest, and it
  says in passing that the output reaches the pad whatever the pad's
  digital mode is.
- **A disabled amplifier drives nothing**, and its output pad then
  floats: it read 943 counts before anything had driven it and held 4095
  after being driven there, which is a floating pad's charge and not a
  level.

## Not covered yet

Driver gaps, each with its reason:

- **The amplifier as an AMPLIFIER.** A follower, a divider or any closed
  loop needs a wire from an output pad to a negative input pad, because
  this block has no internal feedback; with none strapped the only
  honest measurement is the open-loop one above. One jumper - PA2 to
  PA5, or PA3 to PA6 - is what would turn every letter of the suite into
  a gain measurement.
- **The input offset voltage** (1.5 mV typical, 6 mV maximum): a
  property of the closed loop, so the same wire, plus a source between
  the rails to measure it against.
- **The common-mode input range and the drive into a real load** (table
  4-31's CMIR, ILOAD, RLOAD): both want something attached to the output
  pad.
- **The amplifier as a TIMER's input.** The datasheet says the
  comparison result "can be output by GPIO or directly connected to the
  input channel of TIMx"; no register of this chapter selects such a
  route, and the timer chapter's own input tables do not name the
  amplifier. What the sentence means on this family is not established
  here, and the pad path - the output pad IS a timer channel pad on some
  of these pins - is the timer chapter's to measure.

Implemented but not bench-verified, each with what would measure it:

- **The eight parts other than the CH32V203C8.** Which amplifiers a part
  has and which of the twelve pads it bonds fold through its own table,
  and the whole stratum compiles for all nine both ways the hardware
  prologue can be built (`brio check ch32v203`); the twenty-pin part's
  single amplifier is asserted at compile time and measured on no board.
  What would measure them is a board.
