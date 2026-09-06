// mcu: stm32g0b1xx stm32g071xx stm32g031xx
// TIMPCLK is read from the clock tag once, for the prescaler, and no
// timer task has a rebase(): a dynamic clock is refused at compile time
// (docs/design/clock.md, the STM32G0 inventory).
#include "stm32g0/tim.hpp"
struct Dyn {
    static constexpr bool is_static = false;
    static uint32_t hz() { return 16'000'000u; }
    static uint32_t pclk_hz() { return 16'000'000u; }
    template <class U>
    static constexpr bool rebases = false;
};
uint32_t f() { return brio::tim_clock_hz(Dyn{}); }
