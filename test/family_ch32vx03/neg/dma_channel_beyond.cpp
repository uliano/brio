// mcu: ch32v203f6 ch32v203f8 ch32v203g6 ch32v203g8 ch32v203k6 ch32v203k8 ch32v203c6 ch32v203c8 ch32v203rb ch32v303cb ch32v303rb ch32v303rc ch32v303vc
// DMA1 has EIGHT channels on the CH32V203 and seven on the CH32V303 (RM
// table 11-7 stops at DMA1_MADDR8, and the note under every register of
// 11.3 names the classes that have the eighth). A ninth is a register
// that is not there, on every part.
#include "ch32vx03/dma.hpp"

using Beyond = brio::DmaChannel<9>;
void f() { Beyond::stop(); }
