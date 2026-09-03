// mcu: stm32g0b1xx stm32g071xx stm32g031xx
// Figure 271's footnote: LPTIM2 has only input channel 1, so CFGR2's
// IN2SEL field is Reserved there and asking for the second input is a
// configuration that cannot exist.
#include "stm32g0/lptim.hpp"
constexpr brio::LptimConfig cfg{.input2 = brio::LptimInput2::comp2_out};
static_assert(brio::lptim_config_valid(2, cfg), "must be refused");
