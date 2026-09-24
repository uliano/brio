// mcu: ch32v203f6 ch32v203f8 ch32v203g6 ch32v203g8 ch32v203k6 ch32v203k8 ch32v203c6 ch32v203c8 ch32v203rb ch32v303cb ch32v303rb ch32v303rc ch32v303vc
// An engine's ELEMENT is one bus access: PSIZE and MSIZE encode 8, 16
// and 32 bits and nothing else (RM 11.3.3, where the fourth code is
// Reserved). An eight-byte element has no width, and no alignment the
// addresses could be checked against either.
#include "ch32v203/dma.hpp"

using TooWide = brio::DmaTxEngine<7, unsigned long long>;
void f() { TooWide::stop(); }
