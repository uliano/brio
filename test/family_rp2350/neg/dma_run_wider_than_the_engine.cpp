// Must FAIL: an engine's Elem is the WIDEST beat its binding allows, so a
// run of half-words on a byte engine - a PL011's data register, say -
// would write two frames' worth into a register that takes one.
#include "rp2350/dma.hpp"
alignas(2) uint16_t halves[4];
void f() { (void)brio::DmaTxEngine<0, uint8_t>::start(halves, 4); }
