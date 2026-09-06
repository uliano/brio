// mcu: stm32g0b1xx stm32g071xx stm32g031xx
// The ADC derives its prescaler from the clock it is given and has no
// rebase() to follow a change: a dynamic clock (util/clock.hpp's other
// kind - is_static false, hz() a value) is refused at compile time
// rather than read once and left stale (docs/design/clock.md).
#include "stm32g0/adc.hpp"
struct Dyn {
    static constexpr bool is_static = false;
    static uint32_t hz() { return 16'000'000u; }
    static uint32_t pclk_hz() { return 16'000'000u; }
    template <class U>
    static constexpr bool rebases = false;
};
bool f() { return brio::Adc::init(Dyn{}, brio::AdcConfig{}); }
