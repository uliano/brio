// mcu: ch32v203f6 ch32v203f8 ch32v203g6 ch32v203g8 ch32v203k6 ch32v203k8 ch32v203c6 ch32v203c8 ch32v203rb ch32v303cb ch32v303rb ch32v303rc ch32v303vc
// THE BEAT IS THE RUN'S ELEMENT, AND THE BINDING'S ELEMENT IS THE WIDEST
// IT TAKES: a transmit engine bound to a byte register (Elem = uint8_t)
// has no start() for a run of half-words - the 16-bit access would land on
// a register whose data is a byte.
#include <stdint.h>

#include <span>

#include "ch32vx03/dma.hpp"

using Narrow = brio::DmaTxEngine<1, 7, uint8_t>;
alignas(4) uint16_t run[4];
void f() { (void)Narrow::start(std::span<const uint16_t>(run, 4)); }
