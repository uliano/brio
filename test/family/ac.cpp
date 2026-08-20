// AC family smoke TU. The input pin table is the 48/64-pin column,
// identical on DA and DB; the PORTE positives are refused where the
// package lacks PORTE. DACREF arithmetic delegates to util/analog.hpp.
#include "avrdx/ac.hpp"
#include "avrdx/evsys.hpp"

using namespace brio;

void ac_common() {
    (void)Ac<0>::init<AcConfig{.positive = AcPos::ainp3, .negative = AcNeg::dacref,
                               .reference = Ref::v2048, .dacref = 125}>();
    (void)Ac<1>::init({.positive = AcPos::ainp3, .negative = AcNeg::ainn0,
                       .hysteresis = AcHysteresis::small});
    Ac<0>::window<2>(AcWindowSense::outside);
    EventChannel<1>::source(Ac<1>::OutEvent{});
    (void)Threshold<Ac<2>>::init(AcPos::ainp0, 1000, Ref::v2048);
    static_assert(ac_dacref_code(1024, 2048) == 128);
    static_assert(ac_dacref_mv(128, 2048) == 1024);
}

#if defined(__AVR_AVR128DB28__) || defined(__AVR_AVR128DB32__) || \
    defined(__AVR_AVR128DA28__) || defined(__AVR_AVR128DA32__)
// 28/32-pin: the PORTE positives are refused at run time too.
bool ac_small_package() {
    return Ac<0>::init({.positive = AcPos::ainp1});   // PE0: -> false here
}
static_assert(!ac_config_valid(0, AcConfig{.positive = AcPos::ainp1}));
static_assert(!ac_config_valid(2, AcConfig{.positive = AcPos::ainp2}));
#else
static_assert(ac_config_valid(0, AcConfig{.positive = AcPos::ainp1}));
#endif
