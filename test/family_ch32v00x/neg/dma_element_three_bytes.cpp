// mcu: ch32v006k8 ch32v003f4
// A DMA element is one bus access wide - 1, 2 or 4 bytes: a three-byte
// element has no PSIZE/MSIZE code and must be REFUSED.
#include "ch32v00x/dma.hpp"

struct Rgb { uint8_t r, g, b; };
void f() { brio::DmaTxEngine<3, Rgb>::stop(); }
