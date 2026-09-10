// mcu: ch32v006k8
// This family has SPI1 alone: a second instance must be REFUSED.
#include "ch32v00x/spi.hpp"

void f() { brio::Spi<2>::enable(); }
