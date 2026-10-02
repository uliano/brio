// mcu: stm32g0b1xx stm32g071xx stm32g031xx
// A beat is never wider than the engine's element: the element names the
// widest access the bound register takes, and a half-word span handed to
// a byte engine is refused at the call, not stored into a CCR whose
// PSIZE the register does not answer to.
#include <stdint.h>
#include <span>
#include "stm32g0/dma.hpp"
alignas(2) const uint16_t frames[4] = {1, 2, 3, 4};
void f();
void f() { (void)brio::DmaTxEngine<1, 1>::start(std::span<const uint16_t>(frames)); }
