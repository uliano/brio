// mcu: ch32v203f6 ch32v203f8 ch32v203g6 ch32v203g8 ch32v203k6 ch32v203k8 ch32v203c6 ch32v203c8 ch32v203rb ch32v303cb ch32v303rb ch32v303rc ch32v303vc
// A BLOCK ELEMENT WIDER THAN ONE BUS ACCESS. The element type IS the
// beat: CFGR's PSIZE and MSIZE offer a byte, a half-word and a word,
// and a stream of eight-byte elements would have to be two accesses
// per element with nothing in the controller to pair them.
#include "ch32v203/dma.hpp"

using TooWide = brio::DmaPingPongEngine<1, uint64_t>;
void f() { TooWide::stop(); }
