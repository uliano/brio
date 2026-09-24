// mcu: ch32v203f8 ch32v203g6 ch32v203g8 ch32v203k6 ch32v203k8 ch32v203c6 ch32v203c8 ch32v203rb ch32v303cb ch32v303rb ch32v303rc ch32v303vc
// THE FIRST AMPLIFIER, on the twelve parts that have it. Datasheet table
// 2-1 gives the twenty-pin CH32V203F6 ONE operational amplifier, and
// which one it is is in the pin table rather than the count: that
// package bonds neither PB15 nor PB0, which are OPA1's two positive
// inputs, so the amplifier it has is OPA2. `device::has_opa(1)` is that
// fact, `Opa<1>` refuses on the part without it, and this TU is the
// only place OPA1 is named.
#include "ch32v203/opa.hpp"

using namespace brio;

static_assert(device::has_opa(1) && device::has_opa(2));
static_assert(device::opa_count == (device::has_opa(3) ? 4u : 2u));
static_assert(Opa<1>::shift == 0u);

// PB11 and PB15 are OPA1's CHN0 and CHP0 and the smaller packages do
// not bond them either - which is why this TU names the pair that every
// part HERE has (PA6 and PB0 for OPA1's second inputs) and the wider
// packages' own pads are exercised through the pad map alone.
using N1 = OpaIn<1, OpaPin::negative1>;   // PA6
using O1 = OpaOut<1, OpaPin::out0>;       // PA3, ADC channel 3
static_assert(N1::pad == Pad{'A', 6});
static_assert(O1::pad == Pad{'A', 3});
static_assert(O1::adc_channel == 3u);

void opa1_verbs() {
    N1::pin::output(false);
    O1::claim();
    (void)Opa<1>::configure({.positive = OpaPin::positive1,
                             .negative = OpaPin::negative1,
                             .output = OpaPin::out0});
    Opa<1>::enable(true);
    (void)Opa<1>::enabled();
    (void)Opa<1>::field();
    (void)Opa<1>::adc_channel();
    Opa<1>::release();
}

/// The two amplifiers share one register and must not disturb each
/// other: the fields are four bits apart, and each verb keeps the
/// other's.
void opa_two_amplifiers() {
    Opa<1>::enable(true);
    Opa<2>::enable(true);
    (void)Opa<1>::configure(OpaConfig{.output = OpaPin::out1});
    (void)Opa<2>::configure(OpaConfig{.positive = OpaPin::positive1});
    Opa<1>::release();
    Opa<2>::release();
}
