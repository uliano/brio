// OPA family smoke TU: ch32v00x/opa.hpp's verbs instantiated -
// instantiation only, no main(), no hardware. The refusals are pinned
// in the header.
#include "ch32v00x/opa.hpp"

using namespace brio;

void verbs() {
    Opa::unlock();
    (void)Opa::locked();
    (void)Opa::configure({.positive = OpaPositive::pd1, .negative = OpaNegative::pd0, .output = OpaOutput::pd4,
                          .feedback = false, .high_speed = true});
    (void)Opa::configure({.negative = OpaNegative::gain16, .differential = true, .bias = OpaBias::vdd_over_2,
                          .cmp2_reference = OpaCmp2Reference::code1});
    Opa::enable(true);
    (void)Opa::enabled();
    (void)Opa::ctlr1();
    Opa::lock();
    Opa::cmp_unlock();
    (void)Opa::cmp_locked();
    (void)Opa::cmp2_enable(true);
    (void)Opa::cmp2_enabled();
    Opa::cmp_lock();
    (void)opa_positive_pad(OpaPositive::pa2);
    (void)opa_negative_pad(OpaNegative::pa1);
    (void)opa_output_pad(OpaOutput::pa5);
    static_assert(Opa::adc_channel == 9);
}
