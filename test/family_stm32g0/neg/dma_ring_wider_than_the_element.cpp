// mcu: stm32g0b1xx stm32g071xx stm32g031xx
// The receive engine's circular shape takes its beat from the ring's
// element, and a beat is never wider than the engine's: a half-word ring
// bound to a byte engine is refused at the arm, not stored into a CCR
// whose PSIZE the register does not answer to.
#include <stdint.h>
#include "stm32g0/dma.hpp"
uint16_t ring[16];
volatile uint32_t data_register;
void f();
void f() { brio::DmaRxEngine<1, 1>::arm(&data_register, 50, ring); }
