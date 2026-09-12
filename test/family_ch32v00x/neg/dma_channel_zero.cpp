// mcu: ch32v006k8 ch32v003f4
// Channels are numbered from 1 (RM table 8-2): a channel 0 must be
// REFUSED, not wrapped onto the flag registers.
#include "ch32v00x/dma.hpp"

void f() { brio::DmaChannel<0>::stop(); }
