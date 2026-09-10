# OPA (CH32V00x)

The operational amplifier of RM ch. 17 - one OPA with four positive
input pads, a negative input that is a pad or a programmable gain, a
differential PGA, a bias reference, an output on a pad and always on
the ADC's channel 9 - and what of the chapter's comparators this part
has. Documents of record: the CH32V00X reference manual V1.5 (17.2.1
and its sub-sections for the modes, figure 17-1 for the network, 17.3
for the registers), the CH32V006 datasheet V2.0 (2.1 for the parts
the chapter's opening paragraph divides).

## What the silicon does

- **One OPA, locked at reset**: OPA_CTLR1.OPA_LOCK stands until the
  key pair (the flash's, 0x45670123 then 0xCDEF89AB) goes into
  OPA_KEY; written one it locks again until a system reset. CMP_KEY
  and POLL_KEY are the same arrangement for the comparator and the
  polling fields.
- **The inputs**: PSEL1 picks PA2, PD7, PD3 or PD1 for the positive
  side; NSEL1 picks PA1 or PD0 for the negative side, or an internal
  gain of 4, 8, 16 or 32 with the 192 kOhm feedback switched in
  (FB_EN1, mandatory in the PGA modes); PGADIF makes the PGA
  differential with PA4 as its negative side (no gain 32 there); VBEN/
  VBSEL add a bias reference of VDD/2 or VDD/4.
- **The output** goes to PD4, PA5 or nowhere (MODE1), and ALWAYS to
  the ADC's channel 9 and CMP2's positive input.
- **The single-ended PGA's input is high-impedance and the
  differential PGA's is not** (figure 17-1's 3.2 kOhm network,
  measured): a pad's own pull holds the first at a rail and cannot
  hold the second.
- **CMP2 does not enable on the CH32V006** (measured: CMP_KEY lifts
  the lock, CMP_EN2 does not take): the chapter's comparators are the
  CH32V007's, as its opening paragraph says of the CMP module.

## Types and verbs

[brio/ch32v00x/opa.hpp](../../brio/ch32v00x/opa.hpp): `Opa`, a
monostate - `unlock()`/`lock()`/`locked()`, `configure(OpaConfig)`
(the positive and negative selections, the output, the feedback, the
differential mode, the bias, the high-speed mode, CMP2's reference;
refused by `opa_config_valid()` for a gain without the feedback, gain
32 differential, a bias without a gain, or with the block locked),
`enable()`, the CMP2 verbs (`cmp_unlock()`, `cmp2_enable()` answering
whether the bit took), `adc_channel` = 9. `OpaPositive`,
`OpaNegative` (with `opa_gain_of()`), `OpaOutput`, `OpaBias`, and the
pad of each selection (`opa_positive_pad()`, `opa_negative_pad()`,
`opa_output_pad()`, `opa_differential_negative_pad`).

## How to use it

```cpp
brio::Opa::unlock();
brio::Opa::configure({.positive = brio::OpaPositive::pd3, .negative = brio::OpaNegative::gain8});
brio::Opa::enable(true);
brio::Adc::select(brio::AdcInput::opa);              // channel 9: the amplified pad
const uint16_t counts = brio::Adc::read_settled(4);
```

## Bench findings

The reference suite is `test_ch32_opa` (6 verdicts in `z`) on the
CH32V006K8U6, nothing wired.

- **The lock and the keys**: a configuration is refused as found,
  takes after the key pair, and lands as spelled.
- **The PGA into the ADC**: at a gain of 4 with PA2 pulled down,
  channel 9 reads 2 counts; pulled up, 4095; the OPA off, 3.
- **The differential PGA's sign**: P pulled up and N down saturates
  high, N up and P down saturates low; with both at one level the
  reading is neither rail nor the bias (near zero with both up, 0.77
  VDD with both down) - the input network's doing, the finding above.
- **CMP2**: the lock lifts, the enable does not take.

## Not covered yet

Driver gaps, each with its reason:

- The front-end polling (CFGR1/CFGR2: three positive inputs sampled
  in turn on a timer trigger into the ADC, with a window and a reset
  on a fault): the shape of a task this stratum has no user for.
- CMP2's output as TIM1's break source and its filter: the CH32V007's
  comparator, on the part that has it.
- CMP1: the CH32V007's.

Implemented but not bench-verified, each with what would measure it:

- **The gains, the bias reference and the differential transfer**: a
  source on the inputs - a divider on a jumper, or a DAC on a peer -
  read on channel 9 against VREFINT.
- The output on its pads (PD4, PA5) and the high-speed mode's slew:
  a scope on the pad.
- The negative input pads and an external feedback: a resistor across
  the pads.
