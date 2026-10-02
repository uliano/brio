// A receive engine whose widest beat is a byte asked to fill words.
#include "rp2040/dma.hpp"
uint32_t words[4];
void wider() { (void)brio::DmaRxEngine<3>::start(words, 4); }
