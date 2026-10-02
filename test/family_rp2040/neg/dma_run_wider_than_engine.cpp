// A transmit engine whose widest beat is a byte (a PL011's data
// register) handed a run of half-words: the beat is at most Elem.
#include <span>

#include "rp2040/dma.hpp"
uint16_t halves[4];
void wider() { (void)brio::DmaTxEngine<2>::start(std::span<const uint16_t>(halves)); }
