// The element is the widest beat a binding takes: a stream bound to a
// byte-wide register has no half-word start, because a half-word store
// into a byte register writes a datum the peripheral never meant.
// mcu: stm32f429xx stm32f446xx stm32f411xe
#include <stdint.h>
#include <span>
#include "stm32f4/dma.hpp"
using namespace brio;
uint16_t halves[8];
void f() { (void)DmaTxEngine<2, 7, 4>::start(std::span<const uint16_t>(halves)); }
