// OPA family smoke TU: the amplifier every part of the family has, its
// three selections, its pads and the ADC channel its output lands on -
// and on the CH32V303 the other two amplifiers and the high-speed bits.
//
// WHICH AMPLIFIERS A PART HAS is the datasheets' own row: two on every
// CH32V203 but the twenty-pin CH32V203F6, which bonds neither of OPA1's
// positive inputs (PB15 and PB0 are not on that package) and which its
// table therefore counts as having one; four on every CH32V303 (table
// 2-1-1). So OPA2 is here for all thirteen parts, OPA1 is opa1.cpp's - with
// a neg TU proving that naming OPA1 on the smallest part is a compile error
// on the line that asked - and OPA3 and OPA4 are compiled below where the
// part has them.
//
// THE PADS ARE THE CLASS'S and the bonding is the package's, which is
// `Pin`'s own refusal: OpaIn and OpaOut form a Pin from the map, so a
// pad this package does not bring out never compiles. The two series
// share every pad but the second outputs (PB1 and PA4 on the CH32V203,
// PE15 and PE14 on the CH32V303, which are not converter inputs).
#include "ch32vx03/opa.hpp"

using namespace brio;

// ---- the register, and what of it is ours ---------------------------------
static_assert(sizeof(OpaRegs) == 4);
static_assert(opa_base == 0x40023804UL);
// It sits inside the EXTEN block's window, between EXTEN_CTR (0x00) and
// EXTEN_CTR2 (0x08) - which is the reserved word device.hpp shows there.
static_assert(opa_base == hb_base + 0x3800u + 4u);
static_assert(opa_shift_for(1) == 0u && opa_shift_for(2) == 4u);
static_assert(opa_shift_for(3) == 8u && opa_shift_for(4) == 12u);
static_assert(opa_class_has_four == (device::device_class == DeviceClass::v30x_d8));
static_assert(opa_en_bit == 1u && opa_mode_bit == 2u && opa_nsel_bit == 4u && opa_psel_bit == 8u);
static_assert(opa_field_mask == 0xFu);

// ---- the roles of the six pad names ---------------------------------------
static_assert(opa_is_positive(OpaPin::positive0) && opa_is_positive(OpaPin::positive1));
static_assert(opa_is_negative(OpaPin::negative0) && opa_is_negative(OpaPin::negative1));
static_assert(opa_is_output(OpaPin::out0) && opa_is_output(OpaPin::out1));
static_assert(!opa_is_positive(OpaPin::negative0) && !opa_is_output(OpaPin::positive0));
static_assert(opa_code(OpaPin::positive0) == 0u && opa_code(OpaPin::positive1) == 1u);
static_assert(opa_code(OpaPin::negative1) == 1u && opa_code(OpaPin::out1) == 1u);

// ---- the pad map (the CH32V203's 3.2, the CH32V303's table 3-4) ------------
static_assert(opa_pad(1, OpaPin::positive0) == Pad{'B', 15});
static_assert(opa_pad(1, OpaPin::positive1) == Pad{'B', 0});
static_assert(opa_pad(1, OpaPin::negative0) == Pad{'B', 11});
static_assert(opa_pad(1, OpaPin::negative1) == Pad{'A', 6});
static_assert(opa_pad(1, OpaPin::out0) == Pad{'A', 3});
static_assert(opa_pad(1, OpaPin::out1) == (opa_class_has_four ? Pad{'E', 15} : Pad{'B', 1}));
static_assert(opa_pad(2, OpaPin::positive0) == Pad{'B', 14});
static_assert(opa_pad(2, OpaPin::positive1) == Pad{'A', 7});
static_assert(opa_pad(2, OpaPin::negative0) == Pad{'B', 10});
static_assert(opa_pad(2, OpaPin::negative1) == Pad{'A', 5});
static_assert(opa_pad(2, OpaPin::out0) == Pad{'A', 2});
static_assert(opa_pad(2, OpaPin::out1) == (opa_class_has_four ? Pad{'E', 14} : Pad{'A', 4}));
// OPA3 and OPA4 are the CH32V303's (RM 30.3.1's notes name its class):
// mapped there from table 3-4, no pad of theirs elsewhere.
static_assert(opa_pad(3, OpaPin::out0).valid() == opa_class_has_four);
static_assert(!opa_pad(0, OpaPin::out0).valid() && !opa_pad(5, OpaPin::out0).valid());
static_assert(!opa_class_has_four ||
              (opa_pad(3, OpaPin::positive0) == Pad{'B', 13} &&
               opa_pad(3, OpaPin::positive1) == Pad{'C', 5} &&
               opa_pad(3, OpaPin::negative0) == Pad{'B', 2} &&
               opa_pad(3, OpaPin::negative1) == Pad{'C', 2} &&
               opa_pad(3, OpaPin::out0) == Pad{'A', 1} && opa_pad(3, OpaPin::out1) == Pad{'E', 7}));
static_assert(!opa_class_has_four ||
              (opa_pad(4, OpaPin::positive0) == Pad{'B', 12} &&
               opa_pad(4, OpaPin::positive1) == Pad{'C', 4} &&
               opa_pad(4, OpaPin::negative0) == Pad{'B', 1} &&
               opa_pad(4, OpaPin::negative1) == Pad{'C', 3} &&
               opa_pad(4, OpaPin::out0) == Pad{'A', 0} && opa_pad(4, OpaPin::out1) == Pad{'E', 8}));

// ---- the output pad IS the route to the converter, where it is one ----------
static_assert(opa_output_channel(1, OpaPin::out0) == 3u);
static_assert(opa_output_channel(1, OpaPin::out1) == (opa_class_has_four ? 0xFFu : 9u));
static_assert(opa_output_channel(2, OpaPin::out0) == 2u);
static_assert(opa_output_channel(2, OpaPin::out1) == (opa_class_has_four ? 0xFFu : 4u));
static_assert(opa_output_channel(2, OpaPin::negative0) == 0xFFu);
static_assert(opa_output_channel(3, OpaPin::out0) == (opa_class_has_four ? 1u : 0xFFu));
static_assert(opa_output_channel(4, OpaPin::out0) == (opa_class_has_four ? 0u : 0xFFu));
static_assert(opa_output_channel(3, OpaPin::out1) == 0xFFu);

// ---- a configuration, and the four bits it makes ---------------------------
static_assert(opa_config_valid(OpaConfig{}));
static_assert(!opa_config_valid(OpaConfig{.positive = OpaPin::negative0}));
static_assert(!opa_config_valid(OpaConfig{.negative = OpaPin::out1}));
static_assert(!opa_config_valid(OpaConfig{.output = OpaPin::positive1}));
static_assert(opa_config_bits(OpaConfig{}) == 0u);
static_assert(opa_config_bits(OpaConfig{.positive = OpaPin::positive1}) == opa_psel_bit);
static_assert(opa_config_bits(OpaConfig{.negative = OpaPin::negative1}) == opa_nsel_bit);
static_assert(opa_config_bits(OpaConfig{.output = OpaPin::out1}) == opa_mode_bit);

// ---- the amplifier every package of the series carries ---------------------
static_assert(device::has_opa(2));
static_assert(device::opa_count == (device::has_opa(3) ? 4u : device::has_opa(1) ? 2u : 1u));
static_assert(Opa<2>::instance == 2);
static_assert(Opa<2>::shift == 4u);

using P2 = OpaIn<2, OpaPin::positive1>;   // PA7
using N2 = OpaIn<2, OpaPin::negative1>;   // PA5
using O2 = OpaOut<2, OpaPin::out0>;       // PA2, ADC channel 2
static_assert(P2::pad == Pad{'A', 7});
static_assert(N2::pad == Pad{'A', 5});
static_assert(O2::pad == Pad{'A', 2});
static_assert(O2::adc_channel == 2u && O2::reaches_adc);
static_assert(O2::amplifier == 2u);
static_assert(Opa<2>::has_high_speed == opa_class_has_four);

void opa_verbs() {
    // The pads: an input the program holds at a level is driven by the
    // PORT, and only the output is handed to the analog mux.
    P2::pin::output(true);
    N2::pin::output(false);
    O2::claim();

    (void)Opa<2>::configure({.positive = OpaPin::positive1,
                             .negative = OpaPin::negative1,
                             .output = OpaPin::out0});
    (void)Opa<2>::configure({.positive = OpaPin::out0});   // refused: wrong role
    (void)Opa<2>::configuration().output;
    Opa<2>::enable(true);
    (void)Opa<2>::enabled();
    (void)Opa<2>::field();
    (void)Opa<2>::positive(OpaPin::positive0);
    (void)Opa<2>::positive(OpaPin::negative0);   // refused
    (void)Opa<2>::negative(OpaPin::negative0);
    (void)Opa<2>::output(OpaPin::out1);
    (void)Opa<2>::pad(OpaPin::out1);
    (void)Opa<2>::adc_channel();
    (void)Opa<2>::regs().CTLR;
    Opa<2>::release();
}

/// The upper two amplifiers, where the part has them: a template on the
/// instance, so a part without one never instantiates the line. Their
/// CH0 inputs and first output are pads every CH32V303 package bonds
/// (PB13/PB2 into PA1, PB12/PB1 into PA0); the CH1 inputs are port C's,
/// which the LQFP48 does not bring out, and their second outputs port E's.
template <uint8_t n>
void upper_amplifier_verbs() {
    if constexpr (device::has_opa(n)) {
        using A = Opa<n>;
        using Pos = OpaIn<n, OpaPin::positive0>;
        using Neg = OpaIn<n, OpaPin::negative0>;
        using Out = OpaOut<n, OpaPin::out0>;
        static_assert(Out::reaches_adc && Out::adc_channel == (n == 3u ? 1u : 0u));
        static_assert(A::shift == 4u * (n - 1u));
        Pos::pin::output(true);
        Neg::pin::output(false);
        Out::claim();
        (void)A::configure({.positive = OpaPin::positive0,
                            .negative = OpaPin::negative0,
                            .output = OpaPin::out0});
        A::enable(true);
        (void)A::enabled();
        (void)A::field();
        (void)A::configuration();
        (void)A::positive(OpaPin::positive1);
        (void)A::negative(OpaPin::negative1);
        (void)A::output(OpaPin::out1);
        (void)A::pad(OpaPin::out1);
        (void)A::adc_channel();
        A::release();
    }
}

/// The high-speed bits of EXTEN_CTR2, the CH32V303's alone.
template <uint8_t n>
void high_speed_verbs() {
    if constexpr (Opa<n>::has_high_speed) {
        Opa<n>::high_speed(true);
        (void)Opa<n>::high_speed();
        Opa<n>::high_speed(false);
    }
}

void opa_upper_verbs() {
    upper_amplifier_verbs<3>();
    upper_amplifier_verbs<4>();
    high_speed_verbs<2>();
}
